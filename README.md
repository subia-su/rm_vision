# rm_vision · RoboMaster 视觉自瞄系统

> **运行环境**：Ubuntu 20.04 / ROS2 Foxy / Jetson AGX Orin（JetPack 5.1.1）
> **相机**：HikRobot MV-CS016-10UC（海康工业相机，USB3）· **电控**：STM32，串口 `/dev/ttyACM0`
>
> 本项目基于 [chenjunnn/rm_vision](https://github.com/chenjunnn/rm_vision) 二次开发，
> 仓库地址：<https://github.com/subia-su/rm_vision>。
> 本文所有命令均在 Jetson 实机上实测通过。

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
| 装甲板检测 | `armor_detector` | `/detector/armors` | 二值化 + 灯条配对 + 数字分类 + PnP |
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
│   │   ├── armor_detector/          # 装甲板检测（模型 model/mlp.onnx + label.txt）
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
ros2 param get /armor_detector binary_thres
ros2 param set /armor_detector detect_color 1   # 运行时切换蓝方
ros2 param set /armor_detector debug false      # 运行时关闭调试图
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
| `/armor_detector` | `detect_color` | 0 | 0=红方 1=蓝方（电控可经串口下发覆盖） |
| | `binary_thres` | 30 | 灰度阈值：太高漏检、太低噪声多 |
| | `classifier_threshold` | 0.8 | 数字分类置信度 |
| | `debug` | true | **正式运行务必改 false**（见下方警告） |
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

---

## 6. 串口协议（与电控约定）

int16 定点协议（所有物理量 ×100，精度 0.01），定义见
`src/rm_serial_driver/include/rm_serial_driver/packet.hpp`。

| 方向 | 帧头 | 长度 | 结构 |
|---|---|---|---|
| 电控 → 视觉 `ReceivePacket` | `0x5A` | 16 B | 位域(1) + roll,pitch,yaw,aim_x,aim_y,aim_z (6×int16) + CRC16(2) |
| 视觉 → 电控 `SendPacket` | `0xA5` | 26 B | 位域(1) + x,y,z,yaw,vx,vy,vz,v_yaw,r1,r2,dz (11×int16) + CRC16(2) |

- 位域：`ReceivePacket` = detect_color:1 + reset_tracker:1 + reserved:6；
  `SendPacket` = tracking:1 + id:3 + armors_num:3 + reserved:1
- CRC16：多项式 `0x1021`（反射 `0x8408`），初值 `0xFFFF`，小端追加
- 波特率 `115200`，8N1，无流控；帧头不符合时报 `Invalid header`，CRC 失败报 `CRC error!`

> 若电控固件仍是 float32 帧（28 B / 48 B）或 CRC 算法不同，会持续报
> `CRC error!` / `Invalid header`，需两端统一协议。

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
echo 666666 | sudo -S fuser -v /dev/bus/usb/002/003   # 查看占用进程（相机在 Bus 002）
kill <占用进程 PID>                                     # 释放相机
echo 666666 | sudo -S usbreset 002/003                 # 重置相机 USB（或 2bdf:0001）
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
| `CRC error!` / `Invalid header` | 电控协议/波特率与 §6 不符 | 核对固件帧结构；确认 `device_name` |
| 检测输出突然停住 | `debug: true` 拖垮检测器 | `debug` 改 `false` 重启 |
| 跟踪器疯狂 `dropping message` | 无有效电控数据 → 无云台 TF | 先修串口（§7.4） |
| `node list` 空 / 收不到话题 | DDS 共享内存残留 | `ROS_LOCALHOST_ONLY=1` + 清理 `/dev/shm` |
| 启动瞬间 `Message Filter dropping` | TF 尚未就绪 | 正常，1~2 秒自愈；持续则查 §7.4 |
| `Armor jump! / Reset State!` 频繁 | 云台姿态跳变 / 检测抖动 | 查电控 IMU；调 EKF 的 R/Q |

---

## 9. 致谢

- **[chenjunnn / rm_vision](https://github.com/chenjunnn/rm_vision)** —— 原始开源项目，提供检测/跟踪/串口整体架构；
- **[PennState-RoboX / rm_vision](https://github.com/PennState-RoboX/rm_vision)** —— 上游备份与参考实现；
- **HikRobot MVS SDK** —— 海康工业相机驱动支持；
- ROS2 / OpenCV / Eigen / FastRTPS 等开源生态。

本项目仅用于 RoboMaster 竞赛学习与开发。
