// Copyright (c) 2024 rm_vision contributors
// Licensed under the Apache-2.0 License.

#ifndef ARMOR_DETECTOR__YOLO_DETECTOR_HPP_
#define ARMOR_DETECTOR__YOLO_DETECTOR_HPP_

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
/// YOLOv8-OBB 装甲板检测器。
/// 加载 TensorRT FP16 engine，输出与 Detector::detect() 相同结构的 std::vector<Armor>，
/// 以便 armor_detector 节点其余部分（PnP、消息填充、串口下发）完全复用。
///
/// 模型协议：
///   输入  images [1, 3, 640, 640] float32，RGB，除以 255
///   输出  output0 [1, 23, 8400] float32 =
///         [cx, cy, w, h (640 像素空间), 18 类分数(sigmoid), θ(弧度)]
///
/// target_numbers: 允许下发的装甲类型白名单（按 Armor::number 过滤）。
///   "guard" = 哨兵, "1"~"5" = 步兵, "outpost" = 前哨站, "base" = 基地。
///   空集合 = 不过滤，返回全部 18 类。
///   3V3 默认只保留哨兵 + 步兵，避免前哨站/基地干扰跟踪。
class YoloDetector
{
public:
  YoloDetector(
    const std::string & engine_path, float conf_threshold = 0.25f, float nms_threshold = 0.5f,
    const std::vector<std::string> & target_numbers = {});
  ~YoloDetector();

  /// 对一张 RGB 图像推理，返回检测到的装甲板（仅保留与 detect_color 同色的目标）。
  /// detect_color: RED=0（红方，R_* 类）/ BLUE=1（蓝方，B_* 类）。
  std::vector<Armor> detect(const cv::Mat & rgb_img, int detect_color);

private:
  void * impl_;  // PIMPL，隔离 NvInfer/CUDA 头文件
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__YOLO_DETECTOR_HPP_
