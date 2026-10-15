// Copyright (c) 2024 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include "armor_detector/yolo_detector.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <set>
#include <stdexcept>
#include <vector>

namespace rm_auto_aim
{
namespace
{
constexpr int INPUT_W = 640;
constexpr int INPUT_H = 640;
constexpr int NUM_ANCHORS = 8400;
constexpr int NUM_CLASSES = 18;
constexpr int NUM_FEATURES = 4 + NUM_CLASSES + 1;  // 23

struct ClassMeta
{
  const char * number;
  ArmorType type;
};

const ClassMeta CLASS_META[NUM_CLASSES] = {
  {"guard", ArmorType::LARGE},   // 0  B_G 蓝哨兵
  {"1", ArmorType::SMALL},       // 1  B_1
  {"2", ArmorType::SMALL},       // 2  B_2
  {"3", ArmorType::SMALL},       // 3  B_3
  {"4", ArmorType::SMALL},       // 4  B_4
  {"5", ArmorType::SMALL},       // 5  B_5
  {"outpost", ArmorType::SMALL}, // 6  B_O 蓝前哨站
  {"base", ArmorType::SMALL},    // 7  B_Bs 蓝基地小
  {"base", ArmorType::LARGE},    // 8  B_Bb 蓝基地大
  {"guard", ArmorType::LARGE},   // 9  R_G 红哨兵
  {"1", ArmorType::SMALL},       // 10 R_1
  {"2", ArmorType::SMALL},       // 11 R_2
  {"3", ArmorType::SMALL},       // 12 R_3
  {"4", ArmorType::SMALL},       // 13 R_4
  {"5", ArmorType::SMALL},       // 14 R_5
  {"outpost", ArmorType::SMALL}, // 15 R_O 红前哨站
  {"base", ArmorType::SMALL},    // 16 R_Bs 红基地小
  {"base", ArmorType::LARGE},    // 17 R_Bb 红基地大
};

const char * CLASS_NAME[NUM_CLASSES] = {
  "B_G", "B_1", "B_2", "B_3", "B_4", "B_5", "B_O", "B_Bs", "B_Bb",
  "R_G", "R_1", "R_2", "R_3", "R_4", "R_5", "R_O", "R_Bs", "R_Bb"};

class Logger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::fprintf(stderr, "[TRT] %s\n", msg);
    }
  }
};

struct Impl
{
  Logger logger;
  nvinfer1::IRuntime * runtime = nullptr;
  nvinfer1::ICudaEngine * engine = nullptr;
  nvinfer1::IExecutionContext * context = nullptr;
  void * buffers[2] = {nullptr, nullptr};
  float * host_in = nullptr;   // 3*640*640
  float * host_out = nullptr;  // 23*8400
  float conf_th = 0.25f;
  float nms_th = 0.5f;
  std::set<std::string> target_numbers;  // 空 = 不过滤
  std::vector<float> best_score;         // 复用缓冲，避免每帧分配（长度 NUM_ANCHORS）
  std::vector<int> best_cls;
};

struct Detection
{
  float conf;
  int cls;
  float cx, cy, w, h, theta;
};

// 等比缩放 + 114 填充到 640x640，RGB，/255，HWC -> CHW
void preprocess(const cv::Mat & rgb, float * out)
{
  const float scale = static_cast<float>(INPUT_W) / std::max(rgb.cols, rgb.rows);
  const int nw = static_cast<int>(std::lround(rgb.cols * scale));
  const int nh = static_cast<int>(std::lround(rgb.rows * scale));
  cv::Mat resized;
  cv::resize(rgb, resized, cv::Size(nw, nh));
  const int left = (INPUT_W - nw) / 2;
  const int top = (INPUT_H - nh) / 2;

  cv::Mat canvas(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(left, top, nw, nh)));

  // HWC(RGB) -> CHW：cv::split 分离三通道（通道 0/1/2 = R/G/B），连续 memcpy 替代逐像素循环
  cv::Mat canvas_f;
  canvas.convertTo(canvas_f, CV_32FC3, 1.0 / 255.0);
  std::vector<cv::Mat> channels(3);
  cv::split(canvas_f, channels);
  for (int c = 0; c < 3; ++c) {
    std::memcpy(out + c * INPUT_H * INPUT_W, channels[c].data, INPUT_H * INPUT_W * sizeof(float));
  }
}

// 由旋转框 (cx,cy,w,h,θ) 求 4 角点，并按 PnP 需要的顺序排列：
// [左下, 左上, 右上, 右下]（以图像 x 分左右、y 分上下，适用于 |θ|<45°）
void cornersFromXYWHR(
  float cx, float cy, float w, float h, float theta, cv::Point2f out[4])
{
  const float hw = w / 2.0f;
  const float hh = h / 2.0f;
  const float c = std::cos(theta);
  const float s = std::sin(theta);
  // 宽轴 u=(c,s)，高轴 v=(-s,c)
  cv::Point2f pts[4] = {
    cv::Point2f(cx + c * hw - s * hh, cy + s * hw + c * hh),
    cv::Point2f(cx - c * hw - s * hh, cy - s * hw + c * hh),
    cv::Point2f(cx - c * hw + s * hh, cy - s * hw - c * hh),
    cv::Point2f(cx + c * hw + s * hh, cy + s * hw - c * hh)};

  std::sort(pts, pts + 4, [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });
  cv::Point2f left_top = (pts[0].y < pts[1].y) ? pts[0] : pts[1];
  cv::Point2f left_bottom = (pts[0].y < pts[1].y) ? pts[1] : pts[0];
  cv::Point2f right_top = (pts[2].y < pts[3].y) ? pts[2] : pts[3];
  cv::Point2f right_bottom = (pts[2].y < pts[3].y) ? pts[3] : pts[2];

  out[0] = left_bottom;   // 左下
  out[1] = left_top;      // 左上
  out[2] = right_top;     // 右上
  out[3] = right_bottom;  // 右下
}

float boxIoU(const Detection & a, const Detection & b)
{
  const float ax1 = a.cx - a.w / 2, ay1 = a.cy - a.h / 2, ax2 = a.cx + a.w / 2, ay2 = a.cy + a.h / 2;
  const float bx1 = b.cx - b.w / 2, by1 = b.cy - b.h / 2, bx2 = b.cx + b.w / 2, by2 = b.cy + b.h / 2;
  const float iw = std::min(ax2, bx2) - std::max(ax1, bx1);
  const float ih = std::min(ay2, by2) - std::max(ay1, by1);
  const float inter = std::max(0.0f, iw) * std::max(0.0f, ih);
  const float ua = a.w * a.h + b.w * b.h - inter;
  return (ua > 0.0f) ? (inter / ua) : 0.0f;
}

}  // namespace

YoloDetector::YoloDetector(
  const std::string & engine_path, float conf_threshold, float nms_threshold,
  const std::vector<std::string> & target_numbers)
{
  auto * p = new Impl();
  p->conf_th = conf_threshold;
  p->nms_th = nms_threshold;
  p->target_numbers.insert(target_numbers.begin(), target_numbers.end());
  impl_ = p;

  std::ifstream f(engine_path, std::ios::binary);
  if (!f.is_open()) {
    throw std::runtime_error("YoloDetector: cannot open engine " + engine_path);
  }
  f.seekg(0, std::ios::end);
  const size_t size = static_cast<size_t>(f.tellg());
  f.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  f.read(data.data(), static_cast<std::streamsize>(size));

  p->runtime = nvinfer1::createInferRuntime(p->logger);
  p->engine = p->runtime->deserializeCudaEngine(data.data(), size, nullptr);
  if (!p->engine) {
    throw std::runtime_error("YoloDetector: failed to deserialize engine");
  }
  p->context = p->engine->createExecutionContext();

  p->host_in = new float[3 * INPUT_W * INPUT_H];
  p->host_out = new float[NUM_FEATURES * NUM_ANCHORS];
  p->best_score.assign(NUM_ANCHORS, 0.0f);
  p->best_cls.assign(NUM_ANCHORS, -1);
  cudaMalloc(&p->buffers[0], 3 * INPUT_W * INPUT_H * sizeof(float));
  cudaMalloc(&p->buffers[1], NUM_FEATURES * NUM_ANCHORS * sizeof(float));
}

YoloDetector::~YoloDetector()
{
  auto * p = static_cast<Impl *>(impl_);
  if (!p) {
    return;
  }
  cudaFree(p->buffers[0]);
  cudaFree(p->buffers[1]);
  delete[] p->host_in;
  delete[] p->host_out;
  if (p->context) {
    p->context->destroy();
  }
  if (p->engine) {
    p->engine->destroy();
  }
  if (p->runtime) {
    p->runtime->destroy();
  }
  delete p;
}

std::vector<Armor> YoloDetector::detect(const cv::Mat & rgb_img, int detect_color)
{
  auto * p = static_cast<Impl *>(impl_);

  preprocess(rgb_img, p->host_in);
  cudaMemcpy(
    p->buffers[0], p->host_in, 3 * INPUT_W * INPUT_H * sizeof(float), cudaMemcpyHostToDevice);
  p->context->enqueueV2(p->buffers, 0, nullptr);
  cudaMemcpy(
    p->host_out, p->buffers[1], NUM_FEATURES * NUM_ANCHORS * sizeof(float),
    cudaMemcpyDeviceToHost);

  const float * o = p->host_out;
  const float scale = static_cast<float>(INPUT_W) / std::max(rgb_img.cols, rgb_img.rows);
  const int left = (INPUT_W - static_cast<int>(std::lround(rgb_img.cols * scale))) / 2;
  const int top = (INPUT_H - static_cast<int>(std::lround(rgb_img.rows * scale))) / 2;

  // 颜色过滤提前：RED 只遍历 R_*（9..17），BLUE 只遍历 B_*（0..8），比较量减半
  const int cls_begin = (detect_color == RED) ? 9 : 0;
  const int cls_end = (detect_color == RED) ? NUM_CLASSES : 9;

  // 按行遍历（每类一行连续 8400 个 float），cache 友好，替代逐 anchor 跨行访问
  for (int i = 0; i < NUM_ANCHORS; ++i) {
    p->best_score[i] = p->conf_th;
    p->best_cls[i] = -1;
  }
  for (int c = cls_begin; c < cls_end; ++c) {
    const float * row = o + (4 + c) * NUM_ANCHORS;
    for (int i = 0; i < NUM_ANCHORS; ++i) {
      if (row[i] > p->best_score[i]) {
        p->best_score[i] = row[i];
        p->best_cls[i] = c;
      }
    }
  }

  std::vector<Detection> candidates;
  candidates.reserve(64);
  for (int i = 0; i < NUM_ANCHORS; ++i) {
    if (p->best_cls[i] < 0) {
      continue;
    }
    Detection d;
    d.conf = p->best_score[i];
    d.cls = p->best_cls[i];
    d.cx = o[i];
    d.cy = o[NUM_ANCHORS + i];
    d.w = o[2 * NUM_ANCHORS + i];
    d.h = o[3 * NUM_ANCHORS + i];
    d.theta = o[22 * NUM_ANCHORS + i];
    candidates.push_back(d);
  }

  // 按置信度降序贪心 NMS（旋转框用外接矩形 IoU 近似）
  std::sort(candidates.begin(), candidates.end(), [](const Detection & a, const Detection & b) {
    return a.conf > b.conf;
  });
  std::vector<Detection> kept;
  for (const auto & d : candidates) {
    bool overlap = false;
    for (const auto & k : kept) {
      if (k.cls == d.cls && boxIoU(k, d) > p->nms_th) {
        overlap = true;
        break;
      }
    }
    if (!overlap) {
      kept.push_back(d);
    }
  }

  std::vector<Armor> armors;
  for (const auto & d : kept) {
    cv::Point2f corners[4];
    cornersFromXYWHR(d.cx, d.cy, d.w, d.h, d.theta, corners);

    Armor armor;
    armor.left_light.bottom = corners[0];
    armor.left_light.top = corners[1];
    armor.right_light.top = corners[2];
    armor.right_light.bottom = corners[3];
    armor.center = (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0f;
    // 映射回原图坐标
    armor.center = cv::Point2f((armor.center.x - left) / scale, (armor.center.y - top) / scale);
    for (int k = 0; k < 4; ++k) {
      corners[k] = cv::Point2f((corners[k].x - left) / scale, (corners[k].y - top) / scale);
    }
    armor.left_light.bottom = corners[0];
    armor.left_light.top = corners[1];
    armor.right_light.top = corners[2];
    armor.right_light.bottom = corners[3];

    armor.type = CLASS_META[d.cls].type;
    armor.number = CLASS_META[d.cls].number;
    armor.classfication_result = CLASS_NAME[d.cls];
    armor.confidence = d.conf;

    // 3V3 白名单过滤：仅保留目标类型（哨兵 guard / 步兵 1~5），空集合不过滤
    if (
      !p->target_numbers.empty() &&
      p->target_numbers.find(armor.number) == p->target_numbers.end()) {
      continue;
    }
    armors.push_back(armor);
  }
  return armors;
}

}  // namespace rm_auto_aim
