# rm_vision · RoboMaster 视觉自瞄系统

> **运行环境**：Ubuntu 20.04 / ROS2 Foxy / Jetson AGX Orin（JetPack 5.1.1）
> **相机**：HikRobot MV-CS016-10UC（海康工业相机）· **通信**：串口（STM32 电控板）
>
> 本项目基于 [陈君（chenjunnn）的开源项目 rm_vision](https://github.com/chenjunnn/rm_vision) 进行二次开发，
> 二次开发仓库见 <https://github.com/subia-su/rm_vision>。

---

## 1. 功能概述

```
相机采集 ──► 装甲板检测 ──► 目标跟踪(EKF) ──► 串口下发 ──► 电控弹道解算/云台控制
 /image_raw   /detector/armors   /tracker/target    SendPacket(0xA5)
                    ▲                                 │
                    └────── 云台姿态 TF ◄── ReceivePacket(0x5A) ┘
```

- **装甲板检测**：灰度二值化 + 灯条配对 + 数字分类 + PnP 解算，输出装甲板三维位姿
- **目标跟踪**：EKF 估计目标位置/速度/yaw/角速度/半径，处理装甲板跳变与短暂丢失
- **串口通信**：接收电控回传的云台姿态与敌方颜色，下发滤波后的目标状态
- **自瞄闭环在电控端完成**：视觉端只提供目标观测，弹道解算、云台控制由电控（STM32）负责

> 数据性质说明：串口下发的 `SendPacket` 内容是 **跟踪器 EKF 滤波后的目标状态**
> （`/tracker/target`），既不是检测器的原始 PnP 测量值（`/detector/armors`），也不是弹道解算结果。

---

## 2. 项目架构

### 2.1 目录结构

```
.
├── README.md
├── src/
│   ├── rm_auto_aim/                # 自瞄算法
│   │   ├── armor_detector/         #   装甲板检测（二值化/灯条/数字分类/PnP）
│   │   ├── armor_tracker/          #   目标跟踪（EKF + 匹配状态机）
│   │   ├── auto_aim_interfaces/    #   自定义消息（Armor/Armors/Target/TrackerInfo…）
│   │   └── rm_auto_aim/            #   元包（聚合上述三个包）
│   ├── rm_gimbal_description/      # 云台 URDF/xacro（相机安装外参在此注入）
│   ├── rm_serial_driver/           # 串口通信（int16 定点协议 + 广播云台 TF）
│   ├── rm_vision_bringup/          # ★ 启动入口 + 全部参数
│   │   ├── launch/
│   │   │   ├── vision_bringup.launch.py    # 真机入口（相机+检测+串口+跟踪）
│   │   │   ├── no_hardware.launch.py       # 无硬件入口（仅检测+跟踪）
│   │   │   └── common.py                   # 公共节点/参数加载
│   │   └── config/
│   │       ├── node_params.yaml    #   各节点运行时参数（串口/检测/跟踪）
│   │       ├── camera_info.yaml    #   相机内参（标定结果）
│   │       └── launch_params.yaml  #   相机选择 + 相机安装外参
│   ├── ros2_hik_camera/            # 海康相机驱动（内含 MVS SDK 二进制）
│   └── transport_drivers/          # serial_driver 底层（io_context/asio/udp）
└── tools/
    └── camera_monitor.py           # 相机实时监控工具（网页/窗口两种模式）
```

### 2.2 数据流与话题

| 节点 | 输入话题 | 输出话题 | 说明 |
|---|---|---|---|
| `hik_camera` | — | `/image_raw` (`sensor_msgs/Image`) | 海康相机采集（1440×1080） |
| `armor_detector` | `/image_raw` | `/detector/armors` (`Armors`) | 检测装甲板，PnP 解算位姿 |
| `armor_tracker` | `/detector/armors` + TF | `/tracker/target` (`Target`)、`/tracker/info` (`TrackerInfo`) | EKF 跟踪滤波 |
| `rm_serial_driver` | `/tracker/target` | 串口 `SendPacket`、`/latency` | 下发目标状态，广播云台 TF |

其中相机与检测器通过 **ComposableNode 容器 + intra-process 通信** 组合在同一个进程内，降低图像拷贝开销。

### 2.3 TF 坐标树

```
odom ──(floating, 串口驱动广播)──► gimbal_link ──(fixed, 相机安装外参)──► camera_link
                                                                              │
                                                    (fixed, REP-104: RPY -90° 0 -90°)
                                                                              ▼
                                                                   camera_optical_frame
```

- `odom → gimbal_link`：**浮动关节**，由 `rm_serial_driver` 根据电控回传的 roll/pitch/yaw 动态广播（`robot_state_publisher` 不发布浮动关节）。
- `gimbal_link → camera_link`：相机相对云台的**安装外参**，由 `launch_params.yaml` 的 `odom2camera.xyz / rpy` 注入。
- 检测器输出的装甲板位姿在 `camera_optical_frame` 下，跟踪器经 TF 变换到 `odom` 系后再滤波。

### 2.4 自定义消息

| 消息 | 字段 |
|---|---|
| `Armor` | `number`（数字类别）、`type`（small/large）、`distance_to_image_center`、`pose`（相机系位姿） |
| `Armors` | `header` + `Armor[] armors` |
| `Target` | `tracking`、`id`、`armors_num`、`position`、`velocity`、`yaw`、`v_yaw`、`radius_1`、`radius_2`、`dz` |
| `TrackerInfo` | `position_diff`/`yaw_diff`（测量 vs 预测残差）、`position`/`yaw`（未滤波原始值） |

---

## 3. 环境依赖

### 3.1 系统 / 工具

| 依赖 | 说明 | 安装 |
|---|---|---|
| Ubuntu 20.04 + ROS2 Foxy | 基础环境 | 官方 apt 源 |
| `colcon` | 编译工具 | `sudo apt install python3-colcon-common-extensions` |
| HikVision **MVS SDK** | 相机驱动底层 | 已随 `src/ros2_hik_camera/.../hikSDK` 打包（含 amd64/arm64） |
| OpenCV | 图像处理 | JetPack 自带（或 `apt install libopencv-dev`） |
| Eigen3 | 线性代数/EKF | `libeigen3-dev` |

### 3.2 ROS2 软件包

```bash
sudo apt update
sudo apt install -y \
  ros-foxy-cv-bridge ros-foxy-image-transport ros-foxy-image-transport-plugins \
  ros-foxy-camera-info-manager ros-foxy-vision-opencv \
  ros-foxy-tf2-ros ros-foxy-tf2-geometry-msgs ros-foxy-message-filters \
  libasio-dev libeigen3-dev
# 或让 rosdep 自动解析（推荐）：
rosdep install --from-paths src --ignore-src -r -y
```

> `serial_driver`、`io_context`、`asio_cmake_module`、`udp_driver` 已以源码形式放在
> `src/transport_drivers/` 内，随工作区一起编译，**无需额外安装 ros-foxy-serial-driver**。

### 3.3 硬件

| 硬件 | 说明 |
|---|---|
| HikRobot MV-CS016-10UC | USB3 工业相机，1440×1080 |
| STM32 电控板 | USB 转串口，默认枚举为 `/dev/ttyACM0` |
| Jetson AGX Orin | 主控（Ubuntu 20.04 + ROS2 Foxy） |

> 串口权限：把当前用户加入 `dialout` 组 —— `sudo usermod -aG dialout $USER`（重新登录生效）。

---

## 4. 编译

```bash
cd ~/yami                        # 工作区根目录（含 src/）
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

单独编译某个包：

```bash
colcon build --symlink-install --packages-up-to rm_serial_driver   # 连同依赖
colcon build --symlink-install --packages-select armor_detector    # 只编译该包
```

> ⚠️ 使用 `--symlink-install` 后，`config/*.yaml` 改动会直接生效；若为普通拷贝安装，
> 需重新 `colcon build` 或手动同步到 `install/rm_vision_bringup/share/rm_vision_bringup/config/`。

---

## 5. 使用说明

### 5.1 真机运行（默认，接真实相机与电控板）

```bash
source /opt/ros/foxy/setup.bash
source ~/yami/install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

启动内容：`robot_state_publisher` + 相机/检测容器 + `rm_serial_driver`（延时 1.5s 启动）+ `armor_tracker`（延时 2.0s 启动）。
串口与跟踪器延后启动，是为了等相机、检测器和 TF 就绪。

| 项 | 默认值 | 修改位置 |
|---|---|---|
| 相机 | `hik` | `config/launch_params.yaml` → `camera` |
| 串口设备 | `/dev/ttyACM0` | `config/node_params.yaml` → `/serial_driver.device_name` |
| 相机安装外参 | `xyz: 0.10 0.0 0.05`，`rpy: 0.0 0.0 0.0` | `config/launch_params.yaml` → `odom2camera` |
| 相机内参 | 见 `camera_info.yaml` | 重新标定后替换 |

### 5.2 无硬件运行（只跑检测 + 跟踪，不接相机/串口）

```bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```

适用于在 PC 上离线回放图像、调试检测与跟踪算法。

### 5.3 相机实时监控工具

`tools/camera_monitor.py` 是一个独立工具，**只订阅 `/image_raw` 不发布任何话题**，不影响视觉管线，适合在 Jetson 无显示器时远程查看画面：

```bash
source /opt/ros/foxy/setup.bash
python3 tools/camera_monitor.py                 # 网页模式，浏览器打开 http://<jetson-ip>:8080
python3 tools/camera_monitor.py --port 8090     # 指定端口
python3 tools/camera_monitor.py --mode window   # 原生 OpenCV 窗口（需接显示器/X11）
```

### 5.4 常用指令速查

```bash
# —— 环境 ——
source /opt/ros/foxy/setup.bash && source ~/yami/install/setup.bash

# —— 启动 / 停止 ——
ros2 launch rm_vision_bringup vision_bringup.launch.py          # 启动
pkill -f vision_bringup                                          # 停止

# —— 看话题 / 帧率 ——
ros2 topic list
ros2 topic hz /image_raw                                          # 相机帧率
ros2 topic hz /detector/armors                                    # 检测帧率
ros2 topic hz /tracker/target                                     # 跟踪帧率
ros2 topic echo --qos-reliability best_effort /detector/armors    # 检测结果
ros2 topic echo --qos-reliability best_effort /tracker/target     # 跟踪目标
ros2 topic echo /tracker/info                                     # 跟踪器状态（reliable）
ros2 topic echo /latency                                          # 视觉端总延迟(ms)
ros2 run tf2_ros tf2_echo odom gimbal_link                        # 云台姿态

# —— 看/改参数 ——
ros2 param list
ros2 param get /serial_driver device_name
ros2 param get /armor_detector binary_thres
ros2 param set /armor_detector detect_color 1                     # 切蓝方(运行时)
ros2 param set /armor_detector debug false                        # 关调试图

# —— 服务 ——
ros2 service call /tracker/reset std_srvs/srv/Trigger             # 重置跟踪器
```

---

## 6. 串口协议（与电控约定）

本项目使用 **int16 定点协议**（所有物理量 ×100，精度 0.01），与
`src/rm_serial_driver/include/rm_serial_driver/packet.hpp` 严格一致。

| 方向 | 帧头 | 长度 | 结构 |
|---|---|---|---|
| 电控 → 视觉 `ReceivePacket` | `0x5A` | **16 B** | header(1) + 位域(1) + roll,pitch,yaw,aim_x,aim_y,aim_z (6×int16) + CRC(2) |
| 视觉 → 电控 `SendPacket` | `0xA5` | **26 B** | header(1) + 位域(1) + x,y,z,yaw,vx,vy,vz,v_yaw,r1,r2,dz (11×int16) + CRC(2) |

- **位域**：`ReceivePacket` = `detect_color:1` + `reset_tracker:1` + `reserved:6`；
  `SendPacket` = `tracking:1` + `id:3` + `armors_num:3` + `reserved:1`
- **CRC16**：多项式 `0x1021`（反射 `0x8408`），初值 `0xFFFF`，小端追加，覆盖除末 2 字节外的全部字节
- **定点换算**：`int16 = value * 100`（截断），`value = int16 / 100`
- 默认波特率 `115200`，8N1，无流控

> ⚠️ 若电控端仍使用 **float32 协议**（28 B / 48 B），会出现 `CRC error!` / `Invalid header: 00`，两端必须统一。

---

## 7. 参数配置与调优

主要参数集中在 `src/rm_vision_bringup/config/node_params.yaml`。

### 7.1 成像 `/camera_node`

| 参数 | 现值 | 说明 |
|---|---|---|
| `exposure_time` | 6000 | 曝光 μs —— 让灯条灰度接近 255、背景尽量暗 |
| `gain` | 8.0 | 增益，优先调曝光 |
| `camera_info_url` | `package://rm_vision_bringup/config/camera_info.yaml` | 相机内参（标定后替换） |

### 7.2 检测 `/armor_detector`

| 参数 | 现值 | 说明 |
|---|---|---|
| `detect_color` | 0 | 0=红方 / 1=蓝方（电控会经串口下发覆盖） |
| `binary_thres` | 30 | 灰度阈值：太高漏检、太低噪声多 |
| `light.min_ratio` | 0.1 | 灯条 短边/长边 下限 |
| `armor.min_light_ratio` | 0.8 | 两灯条长度比（太严会漏侧视装甲板） |
| `classifier_threshold` | 0.8 | 数字分类置信度 |
| `debug` | true | **比赛时建议改 false**（省 CPU/带宽） |

其余几何约束（`light.max_ratio`=0.4、`light.max_angle`=40、`armor.max_angle`=35 等）在代码里有默认值，一般不动。

### 7.3 跟踪 `/armor_tracker`

EKF 状态向量：`[xc, vxc, yc, vyc, za, vza, yaw, v_yaw, r]`（目标中心位置/速度、yaw/角速度、装甲板半径）。

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

### 7.4 时序 `/serial_driver`

| 参数 | 现值 | 说明 |
|---|---|---|
| `timestamp_offset` | 0.006 | 电控↔视觉固定时延补偿（影响云台 TF 时效） |

> ⚠️ 调参前提：**云台姿态数据必须稳定**。若 `odom→gimbal_link` 的 roll/pitch/yaw 剧烈跳变，
> 跟踪器会频繁 `Armor jump!` / `Reset State!`，此时调 EKF 参数无效，先修云台 TF。

---

## 8. 调试与优化方向

### 8.1 CPU 负载与帧率

自瞄对实时性敏感，CPU 过载会连锁拖低相机/检测/跟踪帧率，造成延迟。

```bash
uptime                                   # 看 1/5/15 分钟负载，理想 < 核数
ps aux --sort=-%cpu | head -20           # 找出 CPU 大户
ros2 topic hz /image_raw /detector/armors /tracker/target   # 各环节帧率是否正常
```

- 已知坑：`robot_state_publisher` 的 `publish_frequency` 若误设 1000 会刷爆 CPU，本项目已改为 **50 Hz**。
- 多次启停后可能残留孤儿 `robot_state_publisher` / 容器进程，检查并清理：
  ```bash
  pgrep -af robot_state_publisher
  pgrep -af component_container
  pkill -f vision_bringup && pkill -f component_container && pkill -f robot_state_publisher
  ```

### 8.2 DDS 发现/通信问题

若节点间收不到数据（话题在但无消息、TF 丢失），多为 DDS 共享内存残留或网络发现异常：

```bash
export ROS_LOCALHOST_ONLY=1            # 本机单机运行，避免走网络发现
# 重启前清理 FastRTPS 共享内存残留（需在停止所有 ROS 节点后执行）
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

### 8.3 延迟定位

视觉端到电控的总延迟由视觉管线延迟 + 串口/电控延迟组成，先定位是哪一段：

```bash
ros2 topic echo /latency        # 视觉端延迟：tracker 目标时间戳 → 串口发送时刻(ms)
ros2 topic hz /image_raw /detector/armors /tracker/target   # 逐级帧率，定位瓶颈
```

- `/latency` 大 → 视觉管线慢（相机低帧率 / 检测器耗时 / CPU 过载）。
- `/latency` 小但仍体感延迟大 → 延迟在电控/串口侧或云台执行侧。

### 8.4 云台反向 / 跟随方向错误

若云台朝装甲板实际运动的**反方向**转，通常不是算法错，而是坐标约定/符号问题：

1. **相机安装外参**：核对 `launch_params.yaml` 的 `odom2camera.rpy` 是否与相机物理朝向一致（默认 `0 0 0`）。
2. **yaw 符号约定**：确认电控回传 yaw / 下发目标 yaw 的正方向约定是否与 REP-103（x 前、y 左、z 上）一致。
3. 排查手段：
   ```bash
   ros2 topic echo --qos-reliability best_effort /tracker/target   # 看 position.y 的符号与目标实际左右是否对应
   ros2 run tf2_ros tf2_echo odom gimbal_link                       # 看云台 TF 姿态
   ```

### 8.5 弹道与命中率

弹道解算、重力补偿、云台控制均在电控端完成。视觉端优化的重点是提供**稳定、低延迟、符号正确**的目标状态：

- 先保证云台姿态稳定（§8.4、§7.4），再调 EKF（§7.3）。
- 命中率问题优先排查：相机内参是否标定准确（`camera_info.yaml`）、外参是否与安装一致、`/latency` 是否超标。

---

## 9. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `Camera failed!` / `Get buffer failed!` | 相机被多个进程占用 | `pkill -f vision_bringup` 后只起一个实例 |
| `CRC error!` / `Invalid header: 00` | 串口协议不匹配（float32 vs int16）或端口错 | 核对 `packet.hpp`；确认 `device_name` |
| `Service not ready, skipping parameter set` | 检测器还没起来（正常，串口先于检测器启动） | 稍等即自动成功；持续则查 `armor_detector` 是否启动 |
| `Message Filter dropping ... 'Unknown'` | 启动瞬间 TF 未就绪 | 属正常，1~2 秒后自愈；持续则查云台 TF |
| `Armor jump!` / `Reset State!` 频繁 | 云台姿态跳变 / 检测抖动 | 查电控 IMU 与云台 TF；调 R/Q |
| 识别不到装甲板 | 曝光太暗 / 阈值不合适 / 无目标 / 颜色不对 | 抓 `binary_img`、`result_img` 看；调 `exposure_time`、`binary_thres`、`detect_color` |
| `pkill` 把自己也杀了 | `pkill -f` 匹配到自身命令行 | 用 `pkill -f '[v]ision_bringup'` 形式 |
| Windows 检出后大量文件"被修改" | Git CRLF 转换 | 仓库已加 `.gitattributes`（`* text=auto eol=lf`） |

---

## 10. 致谢

本项目基于开源项目二次开发，特此致谢：

- **[陈君（chenjunnn）/ rm_vision](https://github.com/chenjunnn/rm_vision)** —— 原始开源项目，提供检测/跟踪/串口整体架构与算法框架。
- **[PennState-RoboX / rm_vision](https://github.com/PennState-RoboX/rm_vision)** —— 上游备份与参考实现。
- **HikRobot MVS SDK** —— 海康工业相机驱动支持。
- **ROS2 / OpenCV / Eigen / FastRTPS** 等开源生态依赖。

> 本项目仅用于 RoboMaster 竞赛学习与开发，二次开发内容与上游项目一致遵循开源协议。
