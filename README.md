# rm_vision · RoboMaster 视觉自瞄系统 使用说明

> 运行环境：**Ubuntu 20.04 / ROS2 Foxy / Jetson AGX Orin**（JetPack 5.1.1）
> 相机：**HikRobot MV-CS016-10UC**（海康工业相机）· 通信：串口（STM32 电控板）
>
> 本项目基于 [陈君（chenjunnn）的开源项目 rm_vision](https://github.com/chenjunnn/rm_vision) 进行二次开发。

---

## 1. 功能概述

```
相机采集 ──► 装甲板检测 ──► 目标跟踪(EKF) ──► 串口下发 ──► 电控弹道解算/云台控制
 /image_raw   /detector/armors   /tracker/target    SendPacket
                     ▲                                   │
                     └────── 云台姿态 TF ◄── ReceivePacket ┘
```

- **装甲板检测**：二值化 + 灯条配对 + 数字分类，输出装甲板三维位姿
- **目标跟踪**：EKF 估计目标位置/速度/yaw/角速度，处理装甲板跳变
- **串口通信**：接收云台姿态/颜色，下发目标状态
- **自瞄闭环**由电控完成，视觉只提供目标观测

---

## 2. 目录结构

```
.
├── README.md                       # 本文档
└── src/
    ├── rm_auto_aim/                # 自瞄算法
    │   ├── armor_detector/         #   装甲板检测（含数字分类模型）
    │   ├── armor_tracker/          #   目标跟踪（EKF + 状态机）
    │   ├── auto_aim_interfaces/    #   自定义消息（Armors/Target/TrackerInfo…）
    │   └── rm_auto_aim/            #   元包
    ├── rm_gimbal_description/      # 云台 URDF（相机安装外参在此）
    ├── rm_serial_driver/           # 串口通信（与电控）
    ├── rm_vision_bringup/          # ★ 启动入口 + 参数
    │   ├── launch/vision_bringup.launch.py
    │   ├── launch/no_hardware.launch.py
    │   └── config/
    │       ├── node_params.yaml    #   真机参数（串口 /dev/ttyACM0）
    │       ├── camera_info.yaml    #   相机内参
    │       └── launch_params.yaml  #   相机选择 + 相机安装外参
    ├── ros2_hik_camera/            # 海康相机驱动（含 MVS SDK 二进制）
    └── transport_drivers/          # serial_driver 底层（io_context/asio）
```

---

## 3. 依赖

### 3.1 系统 / 工具

| 依赖 | 说明 | 安装 |
|---|---|---|
| Ubuntu 20.04 + ROS2 Foxy | 基础环境 | 官方 apt 源 |
| `colcon` | 编译工具 | `sudo apt install python3-colcon-common-extensions` |
| HikVision **MVS SDK** | 相机驱动底层 | 已随 `src/ros2_hik_camera/.../hikSDK` 打包（含 amd64/arm64） |
| OpenCV | 图像处理 | JetPack 自带（或 `apt install libopencv-dev`） |

### 3.2 ROS2 软件包（按包列）

| 包 | 主要依赖 |
|---|---|
| `armor_detector` | rclcpp, rclcpp_components, sensor_msgs, geometry_msgs, visualization_msgs, **message_filters**, **cv_bridge**, **image_transport**, image_transport_plugins, vision_opencv, tf2_geometry_msgs, auto_aim_interfaces |
| `armor_tracker` | rclcpp, rclcpp_components, tf2_ros, tf2_geometry_msgs, auto_aim_interfaces, Eigen3, angles |
| `auto_aim_interfaces` | rosidl_default_generators, std_msgs, geometry_msgs |
| `rm_serial_driver` | rclcpp, rclcpp_components, **serial_driver**, geometry_msgs, tf2_ros, tf2_geometry_msgs, visualization_msgs, std_srvs, auto_aim_interfaces |
| `hik_camera` | rclcpp, rclcpp_components, sensor_msgs, image_transport, image_transport_plugins, camera_info_manager |
| `transport_drivers` | asio, rclcpp, io_context（源码在 `src/` 内，无需 apt 版） |

一次性安装：

```bash
sudo apt update
sudo apt install -y \
  ros-foxy-cv-bridge ros-foxy-image-transport ros-foxy-image-transport-plugins \
  ros-foxy-camera-info-manager ros-foxy-vision-opencv \
  ros-foxy-tf2-ros ros-foxy-tf2-geometry-msgs ros-foxy-message-filters \
  ros-foxy-serial-driver \
  libasio-dev libeigen3-dev
# 或者让 rosdep 自动装：
rosdep install --from-paths src --ignore-src -r -y
```

> Python 依赖（`rclpy`、`cv_bridge`、`numpy`、`opencv`）随 ROS2 / JetPack 提供，无需额外安装。

### 3.3 硬件

| 硬件 | 说明 |
|---|---|
| HikRobot MV-CS016-10UC | USB3 工业相机，1440×1080 |
| STM32 电控板 | 通过 USB 转串口，枚举为 `/dev/ttyACM0` |
| Jetson AGX Orin | 主控（Ubuntu 20.04 + ROS2 Foxy） |

> 串口权限：当前用户需在 `dialout` 组 —— `sudo usermod -aG dialout $USER`（重新登录生效）。

---

## 4. 编译

```bash
cd ~/yami                       # 工作区根目录（含 src/）
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

单独编译某个包：

```bash
colcon build --symlink-install --packages-up-to rm_serial_driver
colcon build --symlink-install --packages-select armor_detector
```

> ⚠️ 修改 `config/*.yaml` 后，若安装目录是拷贝而非软链，记得把文件同步到
> `install/rm_vision_bringup/share/rm_vision_bringup/config/`，或重新 `colcon build`。

---

## 5. 运行

### 5.1 真机运行（默认，接真实电控板）

```bash
source /opt/ros/foxy/setup.bash
source ~/yami/install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

启动内容：`robot_state_publisher` + 相机/检测容器 + `rm_serial_driver` + `armor_tracker`

| 项 | 默认值 | 在哪里改 |
|---|---|---|
| 相机 | `hik` | `config/launch_params.yaml` → `camera` |
| 串口设备 | `/dev/ttyACM0` | `config/node_params.yaml` → `/serial_driver.device_name` |
| 相机安装外参 | `xyz: 0.10 0.0 0.05` | `config/launch_params.yaml` → `odom2camera` |

只跑检测+跟踪（不接相机/串口）：

```bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```

---

## 6. 常用指令速查

```bash
# —— 环境 ——
source /opt/ros/foxy/setup.bash && source ~/yami/install/setup.bash

# —— 启动 / 停止 ——
ros2 launch rm_vision_bringup vision_bringup.launch.py            # 启动
pkill -f vision_bringup                                           # 停止

# —— 看话题 ——
ros2 topic list
ros2 topic hz /image_raw                                          # 相机帧率
ros2 topic echo --qos-reliability best_effort /detector/armors    # 检测结果
ros2 topic echo --qos-reliability best_effort /tracker/target     # 跟踪目标
ros2 topic echo /tracker/info                                     # 跟踪器状态（reliable）
ros2 run tf2_ros tf2_echo odom gimbal_link                        # 云台姿态

# —— 看/改参数 ——
ros2 param list
ros2 param get /serial_driver device_name
ros2 param get /armor_detector binary_thres
ros2 param set /armor_detector detect_color 1                     # 切蓝方(运行时)
ros2 param set /armor_detector debug false                        # 关调试图

# —— 服务 ——
ros2 service call /tracker/reset std_srvs/srv/Trigger             # 重置跟踪器

# —— 查看运行中的串口节点打开了哪个设备 ——
SD=$(pgrep -f '[r]m_serial_driver_node' | head -1); ls -l /proc/$SD/fd | grep tty
```

---

## 7. 串口协议（与电控约定）

本项目使用 **int16 定点协议**（所有物理量 ×100，精度 0.01），与 `src/rm_serial_driver/include/rm_serial_driver/packet.hpp` 严格一致。

| 方向 | 帧头 | 长度 | 结构 |
|---|---|---|---|
| 电控 → 视觉 `ReceivePacket` | `0x5A` | **16 B** | header(1) + 位域(1) + roll,pitch,yaw,aim_x,aim_y,aim_z (6×int16) + CRC(2) |
| 视觉 → 电控 `SendPacket` | `0xA5` | **26 B** | header(1) + 位域(1) + x,y,z,yaw,vx,vy,vz,v_yaw,r1,r2,dz (11×int16) + CRC(2) |

- **位域**：`ReceivePacket` = detect_color:1 + reset_tracker:1 + reserved:6；
  `SendPacket` = tracking:1 + id:3 + armors_num:3 + reserved:1
- **CRC16**：多项式 `0x1021`（反射 `0x8408`），初值 `0xFFFF`，小端追加，覆盖除末 2 字节外的全部
- 定点换算：`int16 = value * 100`（截断），`value = int16 / 100`
- 默认波特率 `115200`，8N1，无流控

> ⚠️ 若电控端仍是 **float32 协议**（28 B / 48 B），会出现 `CRC error!` / `Invalid header: 00`，需要两端统一。

---

## 8. 参数调优要点

主要调参文件：`src/rm_vision_bringup/config/node_params.yaml`

### 8.1 成像 `/camera_node`
| 参数 | 现值 | 说明 |
|---|---|---|
| `exposure_time` | 6000 | 曝光 μs —— 让灯条灰度饱和到 255、背景尽量暗 |
| `gain` | 8.0 | 增益，优先调曝光 |

### 8.2 检测 `/armor_detector`
| 参数 | 现值 | 说明 |
|---|---|---|
| `detect_color` | 0 | 0=红方 / 1=蓝方（电控会经串口下发） |
| `binary_thres` | 30 | 灰度阈值，太高漏检、太低噪声多 |
| `light.min_ratio` | 0.1 | 灯条 短边/长边 下限 |
| `armor.min_light_ratio` | 0.8 | 两灯条长度比（太严会漏侧视） |
| `classifier_threshold` | 0.8 | 数字分类置信度 |
| `debug` | true | **比赛时改 false**（省 CPU/带宽） |

其余几何约束（`light.max_ratio`=0.4、`light.max_angle`=40、`armor.*_center_distance`、
`armor.max_angle`=35）在代码里有默认值，一般不动。

### 8.3 跟踪 `/armor_tracker`
EKF 状态：`[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]`

| 参数 | 现值 | 代码默认 | 说明 |
|---|---|---|---|
| `ekf.sigma2_q_xyz` | 0.05 | 20.0 | 位置过程噪声：↑更跟手、↓更平滑 |
| `ekf.sigma2_q_yaw` | 5.0 | 100.0 | yaw 过程噪声 |
| `ekf.r_xyz_factor` | 4e-4 | 0.05 | 测量噪声：↑更信模型(平滑)、↓更信测量(抖) |
| `ekf.r_yaw` | 5e-3 | 0.02 | yaw 测量噪声 |
| `tracker.max_match_distance` | 0.5 | 0.15 | 匹配位置差阈值(m) |
| `tracker.max_match_yaw_diff` | 1.0 | 1.0 | 匹配 yaw 差阈值(rad) |
| `tracker.tracking_thres` | 5 | 5 | 连续匹配帧数才进入 TRACKING |
| `tracker.lost_time_thres` | 1.0 | 0.3 | 丢失容忍时间(s) |

> 调参口诀：**抖就加 R，滞后就加 Q，丢目标就加匹配阈值，串目标就减匹配阈值**。
> 当前 `r_xyz_factor` 远小于默认值（几乎不平滑），建议从代码默认值起调。

### 8.4 时序 `/serial_driver`
| 参数 | 现值 | 说明 |
|---|---|---|
| `timestamp_offset` | 0.006 | 电控↔视觉固定时延补偿（影响云台 TF 时效） |

> ⚠️ 调参前提：**云台姿态数据必须稳定**。若 `odom→gimbal_link` 的 roll/pitch/yaw 剧烈跳变，
> 跟踪器会频繁 `Armor jump!` / `Reset State!`，此时调 EKF 参数无效。

---

## 9. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `Camera failed!` / `Get buffer failed!` | 相机被多个进程占用 | `pkill -f vision_bringup` 后只起一个实例 |
| `CRC error!` / `Invalid header: 00` | 串口协议不匹配（float32 vs int16）或端口错 | 核对 `packet.hpp`；确认 `device_name` |
| `Service not ready, skipping parameter set` | 检测器还没起来（正常，串口先于检测器启动） | 稍等即自动成功；若持续说明 `armor_detector` 没起来 |
| `Message Filter dropping ... 'Unknown'` | 启动瞬间 TF 未就绪 | 属正常，1~2 秒后自愈；持续则查云台 TF |
| `Armor jump!` / `Reset State!` 频繁 | 云台姿态跳变 / 检测抖动 | 查电控 IMU 与云台 TF；调 R/Q |
| 识别不到装甲板 | 曝光太暗 / 阈值不合适 / 无目标 / 颜色不对 | 抓 `binary_img`、`result_img` 看；调 `exposure_time`、`binary_thres`、`detect_color` |
| `pkill` 把自己也杀了 | `pkill -f` 匹配到自身命令行 | 用脚本执行，或用 `pkill -f '[v]ision_bringup'` |
| Windows 检出后大量文件"被修改" | Git CRLF 转换 | 仓库已加 `.gitattributes`（`* text=auto eol=lf`） |

---

## 10. 相关仓库

- 本项目基于（陈君的开源项目）：<https://github.com/chenjunnn/rm_vision>
- 本项目（二次开发）：<https://github.com/subia-su/rm_vision>
- 上游备份：<https://github.com/PennState-RoboX/rm_vision>
