# rm_vision · RoboMaster 视觉自瞄系统

> **运行环境**：Ubuntu 20.04 / ROS2 Foxy / Jetson AGX Orin（JetPack 5.1.1）
> **相机**：HikRobot MV-CS016-10UC（海康工业相机，USB3）· **电控**：STM32，串口 `/dev/ttyACM0`
>
> 本项目基于 [chenjunnn/rm_vision](https://github.com/chenjunnn/rm_vision) 二次开发，
> 仓库地址：<https://github.com/subia-su/rm_vision>。
> 全部命令与串口通讯补丁均已在 Jetson 实机（ROS2 Foxy）验证通过，实测记录见各章节与 §6.4。

---

## 1. 系统架构

```
HikRobot 相机 ──► 装甲板检测 ──► EKF 跟踪 ──► 串口下发 ──► 电控弹道解算 / 云台控制
  /image_raw     /detector/armors  /tracker/target   SendPacket(0xA5)
                     ▲                                  │
                     └───── 云台姿态 TF ◄── ReceivePacket(0x5A) ┘
```

| 环节 | 包 / 可执行 | 输出 | 说明 |
|---|---|---|---|
| 相机采集 | `hik_camera` | `/image_raw` | 1440×1080，实测约 46 Hz |
| 装甲板检测 | `armor_detector` | `/detector/armors` | YOLOv8-OBB(TensorRT FP16) / 传统 CV 二值化+灯条配对+数字分类+PnP |
| 目标跟踪 | `armor_tracker` | `/tracker/target`、`/tracker/info` | EKF 状态 `[xc vxc yc vyc za vza yaw v_yaw r]` |
| 串口通信 | `rm_serial_driver` | 串口 `SendPacket`、`/latency` | 下发目标状态，广播云台 TF |

> **数据性质**：串口下发的 `SendPacket` 内容是跟踪器 **EKF 滤波后的目标状态**
> （位置/速度/yaw/半径），既不是检测器的原始 PnP 测量值，也不是弹道解算结果。
> 弹道解算与云台控制在电控（STM32）端完成，视觉端只提供目标观测。

TF 树（`odom → gimbal_link → camera_link → camera_optical_frame`）：其中
`odom → gimbal_link` 是浮动关节，由串口驱动按电控回传的 roll/pitch/yaw 动态广播；
其余两级是固定关节，相机安装外参由 `launch_params.yaml` 注入。

---

## 2. 目录结构

```
~/yami/                              # 工作区根目录
├── README.md
├── src/
│   ├── rm_auto_aim/
│   │   ├── armor_detector/          # 装甲板检测（YOLOv8-OBB engine + 传统 CV mlp.onnx/label.txt）
│   │   ├── armor_tracker/           # EKF 跟踪 + 匹配状态机
│   │   ├── auto_aim_interfaces/     # 自定义消息 Armor/Armors/Target/TrackerInfo…
│   │   └── rm_auto_aim/             # 元包
│   ├── rm_gimbal_description/       # 云台 URDF（urdf/rm_gimbal.urdf.xacro）
│   ├── rm_serial_driver/            # 串口驱动（int16 定点协议，packet.hpp）
│   ├── rm_vision_bringup/           # ★ 启动入口 + 全部参数
│   │   ├── launch/vision_bringup.launch.py    # 真机入口
│   │   ├── launch/no_hardware.launch.py       # 无硬件入口
│   │   └── config/
│   │       ├── node_params.yaml     # 各节点运行时参数
│   │       ├── launch_params.yaml   # 相机选择 + 相机安装外参
│   │       └── camera_info.yaml     # 相机内参（标定结果）
│   ├── ros2_hik_camera/             # 海康相机驱动（包名 hik_camera，内含 MVS SDK）
│   └── transport_drivers/           # serial_driver 底层（io_context/asio/udp）
└── tools/
    └── camera_monitor.py            # 相机实时监控（网页 / 窗口两种模式）
```

---

## 3. 依赖与编译

```bash
# 依赖（或直接 rosdep install）
sudo apt install -y \
  ros-foxy-cv-bridge ros-foxy-image-transport ros-foxy-image-transport-plugins \
  ros-foxy-camera-info-manager ros-foxy-vision-opencv \
  ros-foxy-tf2-ros ros-foxy-tf2-geometry-msgs ros-foxy-message-filters \
  libasio-dev libeigen3-dev

# 编译（实测：12 packages finished, 0 failures）
cd ~/yami
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

> `--symlink-install` 安装为软链，修改 `src/…/config/*.yaml` 直接生效，无需重新编译。
> `transport_drivers` 以源码内嵌，无需安装 ros-foxy-serial-driver。
> 串口权限：`sudo usermod -aG dialout $USER`（重新登录生效）。

---

## 4. 使用方法

> 单机运行建议在所有终端统一设置 `export ROS_LOCALHOST_ONLY=1`（避免 DDS 走网口发现，
> 见 §7.1）。以下命令均带此设置。

### 4.1 完整自瞄（真机，相机 + 电控都在）

```bash
cd ~/yami
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

启动内容：`robot_state_publisher` + 相机/检测容器 + 串口驱动（延时 1.5s）+ 跟踪器（延时 2.0s），
后两者延后启动是为了等相机、检测器与 TF 就绪。停止：`Ctrl-C`（或 `pkill -f vision_bringup`）。

### 4.2 只看相机画面（网页实时监控，无需显示器）

```bash
# 终端 1：只起相机
cd ~/yami
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch hik_camera hik_camera.launch.py

# 终端 2：起监控网页（只订阅 /image_raw，不影响管线）
cd ~/yami
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=1
python3 tools/camera_monitor.py

# 本机浏览器打开： http://<jetson-ip>:8080/
```

实测：页面实时显示画面与 FPS；接口 `/status`（JSON 状态）、`/snapshot.jpg`（单帧）可用。
可选参数：`--port 8090` 换端口；`--mode window` 改为原生 OpenCV 窗口（需接显示器）。

### 4.3 无硬件运行（只跑检测 + 跟踪）

```bash
cd ~/yami
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```

### 4.4 常用查看指令（均实测可用）

```bash
cd ~/yami && source /opt/ros/foxy/setup.bash && source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 node list                                  # 节点列表
ros2 topic list                                 # 话题列表
ros2 topic hz /image_raw                        # 相机帧率（实测 ~46 Hz）
ros2 topic hz /detector/armors                  # 检测帧率
ros2 topic hz /tracker/target                   # 跟踪帧率
ros2 topic echo /detector/armors                # 检测结果（含每块装甲板位姿）
ros2 topic echo /tracker/target                 # 跟踪目标（位置/速度/yaw/半径）
ros2 topic echo /tracker/info                   # 跟踪器状态与原始残差
ros2 topic echo /latency                        # 视觉端总延迟 (ms)
ros2 run tf2_ros tf2_echo odom gimbal_link      # 云台姿态（电控数据有效时才有）
ros2 service call /tracker/reset std_srvs/srv/Trigger   # 重置跟踪器

# 参数
ros2 param get /serial_driver device_name
ros2 param get /armor_detector use_yolo         # 检测模式（true=YOLO）
ros2 param get /armor_detector yolo.target_numbers
ros2 param get /armor_detector binary_thres     # 仅传统 CV 模式有效
ros2 param set /armor_detector detect_color 1   # 运行时切换蓝方（两种模式共用）
ros2 param set /armor_detector debug false      # 运行时关闭调试图（仅传统 CV）

# 串口收发统计（每秒 Feedback frames / CRC errors，需先停掉完整视觉，避免串口被占用）
ros2 run rm_serial_driver rm_serial_driver_node --ros-args -r __node:=serial_driver \
  --log-level debug --params-file ~/yami/src/rm_vision_bringup/config/node_params.yaml
```

> 注意：Foxy 的 `ros2 topic hz/echo` **不支持** `--qos-reliability` 等 QoS 选项，
> 直接使用即可（本仓库发布端为 best-effort，CLI 默认 QoS 可正常接收）。

---

## 5. 参数配置

主参数文件：`src/rm_vision_bringup/config/node_params.yaml`（软链安装，改后重启生效）。

| 节点 | 参数 | 现值 | 说明 |
|---|---|---|---|
| `/camera_node` | `exposure_time` | 6000 | 曝光 μs，让灯条饱和、背景压暗 |
| | `gain` | 8.0 | 增益，优先调曝光 |
| `/serial_driver` | `device_name` | `/dev/ttyACM0` | 串口设备 |
| | `timestamp_offset` | 0.006 | 电控↔视觉固定时延补偿 |
| `/armor_detector` | `use_yolo` | true | true=YOLOv8-OBB(TensorRT)；false=传统 CV |
| | `yolo.conf_threshold` | 0.25 | YOLO 置信度阈值 |
| | `yolo.nms_threshold` | 0.5 | YOLO NMS IoU 阈值 |
| | `yolo.target_numbers` | `[guard,1..5]` | 3V3 白名单：哨兵 guard + 步兵 1~5；`[]`=全部 18 类 |
| | `detect_color` | 0 | 0=红方 1=蓝方（两种模式共用，电控可经串口下发覆盖） |
| | `binary_thres` | 30 | 灰度阈值：太高漏检、太低噪声多（仅传统 CV） |
| | `classifier_threshold` | 0.8 | 数字分类置信度（仅传统 CV） |
| | `debug` | false | 仅在调图时开启（仅传统 CV 有效）；默认关闭以减少处理开销 |
| `/armor_tracker` | `ekf.sigma2_q_xyz` | 0.05 | 位置过程噪声：↑跟手 ↓平滑 |
| | `ekf.r_xyz_factor` | 4e-4 | 测量噪声：↑信模型 ↓信测量 |
| | `tracker.max_match_distance` | 0.5 | 匹配位置差阈值 (m) |
| | `tracker.lost_time_thres` | 1.0 | 丢失容忍时间 (s) |

相机安装外参在 `src/rm_vision_bringup/config/launch_params.yaml` 的 `odom2camera`
（默认 `xyz "0.10 0.0 0.05"`、`rpy "0.0 0.0 0.0"`，须与相机实际安装朝向一致）。
相机内参在 `src/rm_vision_bringup/config/camera_info.yaml`（重新标定后替换）。

> ⚠️ **debug 实测警告**：`debug: true` 时检测器每帧都要编码 JPEG/Theora 多路 1080p 调试图，
> 相机满帧（~46 Hz）输入下检测器单线程执行器会被拖垮，**检测输出停滞**
> （此时连 `ros2 param set` 都会因执行器忙而超时）。正式运行请在
> `node_params.yaml` 中把 `debug` 改为 `false` 后启动；调试完图像参数后再按需打开。

> 上述 `debug` 调试图开销警告仅针对传统 CV 模式；YOLO 模式（`use_yolo: true`）不发布调试图。

### 5.1 YOLOv8-OBB 检测器（`use_yolo: true`，默认）

检测器默认走 YOLOv8s-OBB + TensorRT FP16，替代传统 CV 的灯条配对。要点：

- **模型**：`yolov8s_obb_fp16.engine`（~23 MiB，输入 640×640，输出 `[1,23,8400]`，
  其中 23 = cx/cy/w/h + 18 类分数 + θ）。18 类按颜色分区：`B_*`(0~8) / `R_*`(9~17)；
  `G`=哨兵(LARGE 225×55)、`1~5`=步兵(SMALL 135×55)、`O`=前哨站(SMALL)、`Bs/Bb`=基地(SMALL/LARGE)。
- **engine 构建**（在训练机导出 `best.onnx` 后，于 Jetson 上执行）：
  ```bash
  /usr/src/tensorrt/bin/trtexec --onnx=/tmp/yolov8s.onnx \
    --saveEngine=/tmp/yolov8s_obb_fp16.engine --fp16 \
    --memPoolSize=workspace:2048 --tacticSources=-CUDNN --iterations=100
  cp /tmp/yolov8s_obb_fp16.engine \
    ~/yami/src/rm_auto_aim/armor_detector/model/yolov8s_obb_fp16.engine
  ```
  实测 Xavier（TRT 8.5.2 / CC 7.2）：构建约 21 分钟，GPU 推理约 16.8 ms（58.75 qps）。
- **3V3 白名单**：`yolo.target_numbers` 默认 `["guard","1","2","3","4","5"]` 只保留哨兵 + 步兵；
  置空 `[]` 则识别全部 18 类（含前哨站/基地）。
- **颜色过滤**：`detect_color` 在 YOLO 模式同样生效（RED=0 只出 `R_*`，BLUE=1 只出 `B_*`）。
- 两种模式共用 `detect_color`；`binary_thres`/`classifier_threshold`/`debug` 仅传统 CV 生效。

---

## 6. 串口协议（与当前步兵下位机核对）

协议定义：`src/rm_serial_driver/include/rm_serial_driver/packet.hpp`；步兵下位机对应
`app/gimbal/vision.c` 的 `Aimbot_FeedUsb / AimbotFeedback_Update`。
当前两端均为 **int16 ×100、数值和 CRC 小端**，不能混用旧 float32 帧。
帧长包含帧头、标志、负载和 CRC；字节偏移从 0 开始。

| 方向 | 帧头 | 总长度 | 完整组成 |
|---|---|---|---|
| 电控 → 视觉 `ReceivePacket` | `0x5A` | 16 B | 帧头(1) + flags(1) + 6×int16(12) + CRC16(2) |
| 视觉 → 电控 `SendPacket` | `0xA5` | 26 B | 帧头(1) + flags(1) + 11×int16(22) + CRC16(2) |

### 6.1 标志、偏移与物理单位

- 上行 flags（偏移 1）：bit0 `detect_color`（0 红、1 蓝），bit1 `reset_tracker`，bit2～7 保留为 0。
- 下行 flags（偏移 1）：bit0 `tracking`，bit1～3 `id`，bit4～6 `armors_num`，bit7 保留为 0。
  例如 tracking=1、id=4、armors_num=4 时 flags=`0x49`；不是将数量左移 3 位。
- `id`：0 前哨站（未跟踪时空 ID 也编码为 0），1～5 编号，6 哨兵，7 基地。
  `armors_num` 在有效跟踪时只能是 2、3 或 4；未锁定时可以为 0。
- 发送整数 = 物理量 ×100；接收物理量 = 有符号整数 ÷100。
  所有 float 字段采用相同缩放，但**单位不同**：角度 rad、角速度 rad/s、位置 m、速度 m/s。
  int16 范围对应 -327.68～327.67；视觉下行保持原来的向零截断，上行由固件四舍五入。

| 帧 | 字节偏移 | 字段 | 单位 / 语义 |
|---|---|---|---|
| 上行 | 2 / 4 / 6 | roll / pitch / yaw | rad，送入 `tf2::Quaternion::setRPY`，不能传度数 |
| 上行 | 8 / 10 / 12 | aim_x / aim_y / aim_z | m，调试瞄准点；当前步兵固件发送 0，不代表没有目标 |
| 上行 | 14～15 | CRC16 | 对偏移 0～13 计算，低字节在前 |
| 下行 | 2 / 4 / 6 | x / y / z | m，`odom` 中目标中心；z 是当前被跟踪装甲板高度 |
| 下行 | 8 | yaw | rad，目标装甲板朝向，不是云台 Yaw 指令；连续 EKF yaw 按等价周期角发送 |
| 下行 | 10 / 12 / 14 | vx / vy / vz | m/s，目标运动速度 |
| 下行 | 16 | v_yaw | rad/s，目标角速度 |
| 下行 | 18 / 20 / 22 | r1 / r2 / dz | m，装甲板到中心半径及另一组装甲板高度差 |
| 下行 | 24～25 | CRC16 | 对偏移 0～23 计算，低字节在前 |

坐标约定：`odom` 为 x 前、y 左、z 上；相机光学坐标为 x 右、y 下、z 前，必须经过 TF 转换。
正 Yaw 从上方看为逆时针；ROS 正 Pitch 绕 +y，使前向轴朝下。
因此下位机几何角为 `yaw=atan2(y,x)`、`pitch=-atan2(z,hypot(x,y))`。
DM4310 的机械正方向还取决于安装与固件映射，不能仅凭正负号修改电机参数。

CRC：多项式 `0x1021`，按位右移实现使用反射多项式 `0x8408`，初值 `0xFFFF`，
**无最终异或**，校验覆盖帧头、flags 和全部负载。参考：ASCII `123456789` 的 CRC=`0x6F91`。

### 6.2 接口、分包与频率

视觉端连接云台 C 板**原生 USB CDC**（通常 `/dev/ttyACM0`），配置 115200、8N1、无流控。
115200 是本工程串口节点与 CDC line-coding 的约定，不是 UART 物理链路速度；它也不限制 USB
每次读出的字节数。VT03 遥控串口、DAPLink 附带 COM 口与 VOFA 调试串口都是其他接口。

USB/串口一次读写可能只传输部分帧，也可能合并多帧。修复后的驱动按实际返回长度累积、CRC
校验后广播 TF，坏帧搜索下一个 `0x5A`；下行持续写完 26 字节。目标订阅仅保留最新一条。
上行默认每 10 ms 尝试发送一次（约 100 Hz）；下行随 `/tracker/target` 更新，不能按固定 USB 块长解析。
损坏包统计（每秒 `Feedback frames=..., CRC errors=...`）需要 debug 级别日志，见 §4.4 的
串口独立运行命令。注意：Foxy 的 rcl **不支持** `--log-level serial_driver:=debug` 这种
`logger:=level` 写法，须使用 `--log-level debug`。统计最多每秒打印一次；不再逐帧刷
CRC/Invalid header 日志。实测（步兵下位机）约 100 Hz 有效帧、CRC errors=0。

### 6.3 当前下位机能力与限制

当前 `AimbotSolver_Update` 校验 tracking、2/3/4 块装甲板、IMU、几何与 100 ms 本地包龄后，
按速度和**收包后经过的时间**外推，再解算第 0 块装甲板：
`xa=xc-r1*cos(yaw)`、`ya=yc-r1*sin(yaw)`。
两轴使用同一目标模型，方向公式与上述 ROS 坐标约定一致。

- 固件已接收 r2/dz，但当前简化解算未使用它们，也未做最优装甲板切换。
- 当前没有弹速、重力或弹丸飞行时间补偿；“协议已匹配”不表示完整火控弹道已完成。
- 26 字节协议没有时间戳或序列号，MCU 的包龄不是相机曝光到电机的总延迟。
  `/latency` 是视觉电脑从目标消息时间戳到串口写完的时间，不测量 USB 到 MCU 的传输延迟。
- ×100 的角度分辨率为 0.01 rad（约 0.573°），半径/位置分辨率为 1 cm；精瞄会受量化影响。
  若将来升级精度，必须两端同时引入协议版本，不能只改任意一端缩放或换 float32。
- 上位机 `tracking_thres=5` 且源码以 `detect_count > tracking_thres` 转入锁定，需初始化后
  连续 6 次匹配；首次看到识别框不会立即输出 tracking=1。TEMP_LOST 可以继续发预测目标。
  本次保持跟踪阈值、EKF 与 MCU PID/限速参数原样。

先确认下位机 `vision_frames` 持续增长、`vision_tracking=1`、`tracking=1`（有效解算）、
`vision_age_ms <= 100`、CRC 错误不增长，再看 Yaw/Pitch 误差是否趋零。
故障、STOP、两轴使能与 Yaw 防发散状态必须同时查看。

### 6.4 回归与部署

固定参考目标帧（tracking=1，id=4，armors_num=4；x=5.00m、y=-1.00m）：

```text
A5 49 F4 01 9C FF 32 00 9D 00 19 00 CE FF 00 00 78 00 1A 00 1E 00 05 00 14 C5
```

上行参考帧（detect_color=1 蓝色，reset=1，roll=0.12rad、pitch=-0.20rad、yaw=1.00rad）：

```text
5A 03 0C 00 EC FF 64 00 7B 00 38 FE 15 03 07 2B
```

`serial_packet_stream_test` 覆盖拆包、截断/CRC 恢复、连续帧、短写、上述参考帧、位域、
负数与越界/非有限数值。部署在视觉电脑工作区根目录：

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select rm_serial_driver rm_vision_bringup
colcon test --packages-select rm_serial_driver
colcon test-result --verbose
source install/setup.bash
# 先停止旧视觉进程，再使用 §4.1 启动完整自瞄
```

本次默认关闭检测调试图以减少处理开销。无需因这份上位机补丁重新烧录 MCU。

已在 Jetson（ROS2 Foxy）实测验证：

- 完整编译 12 个包 0 失败；`colcon test --packages-select rm_serial_driver` 的
  24 项测试（含 `serial_packet_stream_test` 与各 lint）全部通过。
- 实机串口（步兵下位机）实测约 100 Hz 有效反馈帧、`CRC errors=0`，云台 TF 正常广播。

实车延迟与高负载下的稳定性仍待实车进一步确认。

---

## 7. 调试与优化（本机实测经验）

### 7.1 DDS 发现异常（node list 空 / topic 无数据）

```bash
export ROS_LOCALHOST_ONLY=1     # 单机运行走 localhost，避免网口发现
# 重启所有 ROS 进程后清理共享内存残留：
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

实测：不清理残留时新起进程 `ros2 node list` 为空、订阅不到数据；清理后恢复。

### 7.2 CPU 负载与帧率

```bash
uptime                                   # 负载应低于核心数
ps aux --sort=-%cpu | head -15
ros2 topic hz /image_raw /detector/armors /tracker/target
```

- 多次启停会残留孤儿 `robot_state_publisher` / `component_container`，占用相机与 CPU：
  `pkill -f vision_bringup; pkill -f component_container; pkill -f robot_state_publisher`
- 检测器停滞优先怀疑 `debug: true`（见 §5 警告），关掉后帧率立刻恢复。

### 7.3 相机打不开 / Get buffer failed

日志出现 `Get buffer failed! nRet: [80000003]`（MVS 错误码 `MV_E_CALLORDER`）→
**有旧进程仍占用相机**（旧 vision_bringup 的容器占着 USB 设备）：

```bash
sudo fuser -v /dev/bus/usb/002/003   # 查看占用进程（相机在 Bus 002）
kill <占用进程 PID>                                     # 释放相机
sudo usbreset 002/003                 # 重置相机 USB（或 2bdf:0001）
```

### 7.4 跟踪器一直丢弃消息（`Message Filter dropping … 'Unknown'`）

原因是 TF 链 `odom → gimbal_link` 缺失：该浮动关节由串口驱动在**收到有效电控数据**后才广播。
先解决串口（§6 协议、波特率、`/dev/ttyACM0` 是否被占用），
确认 `ros2 run tf2_ros tf2_echo odom gimbal_link` 能输出姿态，再排查跟踪。

### 7.5 延迟定位

```bash
ros2 topic echo /latency        # 视觉端延迟：跟踪目标时间戳 → 串口发送时刻 (ms)
ros2 topic hz /image_raw /detector/armors /tracker/target   # 逐级帧率定位瓶颈
```

`/latency` 大 → 视觉管线慢（低帧率/过载）；`/latency` 小仍体感延迟 → 电控/串口侧。

### 7.6 云台反向 / 跟错方向

1. 核对 `launch_params.yaml` 的 `odom2camera.rpy` 是否与相机物理安装朝向一致；
2. 核对电控回传 yaw 与下发目标 yaw 的正方向约定（REP-103：x 前、y 左、z 上）；
3. `ros2 topic echo /tracker/target` 看 `position.y` 符号与目标实际左右是否一致。

---

## 8. 常见问题速查

| 现象 | 原因 | 处理 |
|---|---|---|
| `Get buffer failed! / Camera failed!` | 相机被旧进程占用 | 按 §7.3 杀进程 + usbreset |
| 反馈 CRC 统计持续增长 / 无有效反馈 | 协议、接口或数据流不符 | 按 §6 核对 16/26 字节帧、CRC 与原生 USB 设备 |
| 检测输出突然停住 | `debug: true` 拖垮检测器 | `debug` 改 `false` 重启 |
| 跟踪器疯狂 `dropping message` | 无有效电控数据 → 无云台 TF | 先修串口（§7.4） |
| `node list` 空 / 收不到话题 | DDS 共享内存残留 | `ROS_LOCALHOST_ONLY=1` + 清理 `/dev/shm` |
| 启动瞬间 `Message Filter dropping` | TF 尚未就绪 | 正常，1~2 秒自愈；持续则查 §7.4 |
| 启动报 `YoloDetector: cannot open engine` / `failed to deserialize` | engine 未构建或路径不对 | 按 §5.1 构建并把 engine 放到 `model/` 目录 |
| `Armor jump! / Reset State!` 频繁 | 云台姿态跳变 / 检测抖动 | 查电控 IMU；调 EKF 的 R/Q |

---

## 9. 致谢

- **[chenjunnn / rm_vision](https://github.com/chenjunnn/rm_vision)** —— 原始开源项目，提供检测/跟踪/串口整体架构；
- **[PennState-RoboX / rm_vision](https://github.com/PennState-RoboX/rm_vision)** —— 上游备份与参考实现；
- **[xkddyl / daoshangfei](https://github.com/xkddyl)** —— 串口分包重同步、协议校验、显式小端编解码与独立回归测试（PR #1）；
- **HikRobot MVS SDK** —— 海康工业相机驱动支持；
- ROS2 / OpenCV / Eigen / FastRTPS 等开源生态。

本项目仅用于 RoboMaster 竞赛学习与开发。
