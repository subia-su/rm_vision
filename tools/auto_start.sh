#!/bin/bash
# rm_vision 开机自启动：自瞄管线 + 网页 YOLO 检测框可视化
# 由 systemd 服务 rm_vision.service 调用（User=kami）
# 日志：~/yami/logs/vision_bringup.log、~/yami/logs/visualizer.log

LOG_DIR="$HOME/yami/logs"
mkdir -p "$LOG_DIR"

log() { echo "[$(date '+%F %T')] $*"; }

stop_all() {
  pkill -f vision_bringup 2>/dev/null
  pkill -f armor_visualizer 2>/dev/null
  pkill -f _ros2_daemon 2>/dev/null
  sleep 2
}

start_bringup() {
  cd "$HOME/yami"
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  # 注意：不设 ROS_LOCALHOST_ONLY（设了会导致话题数据收不到）
  ros2 launch rm_vision_bringup vision_bringup.launch.py >> "$LOG_DIR/vision_bringup.log" 2>&1
}

start_visualizer() {
  cd "$HOME/yami"
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  while true; do
    log "starting armor_visualizer" >> "$LOG_DIR/visualizer.log"
    python3 tools/armor_visualizer.py --mode web --port 8081 >> "$LOG_DIR/visualizer.log" 2>&1
    log "visualizer exited, restarting in 3s" >> "$LOG_DIR/visualizer.log"
    sleep 3
  done
}

# ---- 主流程 ----
stop_all
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

log "=== auto start begin ===" >> "$LOG_DIR/vision_bringup.log"

start_bringup &
BRINGUP_PID=$!
log "vision_bringup pid=$BRINGUP_PID" >> "$LOG_DIR/vision_bringup.log"

# 等管线就绪（相机初始化 + 检测器加载 engine + 串口/跟踪器延时启动）
sleep 15

start_visualizer &
log "armor_visualizer started" >> "$LOG_DIR/vision_bringup.log"

# 主进程跟随 bringup：bringup 异常退出则本脚本退出，
# systemd 按 Restart=on-failure 在 5s 后重启整个服务（含可视化）
wait $BRINGUP_PID
log "vision_bringup exited, script ending (systemd will restart)" >> "$LOG_DIR/vision_bringup.log"
