#!/bin/bash
# 【讲解】Jetson 上后台启动实验三仿真的小脚本（可选，也可以直接 ros2 launch）。
# 用法：./run_sim.sh [scenario=normal|abnormal] [gui=true|false] [其它 launch 参数...]
#   例：./run_sim.sh normal false            无头跑正常场景
#       ./run_sim.sh abnormal true           带 Gazebo 画面跑异常场景（需 DISPLAY=:1）
# 日志：~/exp3_logs/launch_<scenario>_<时间>.log；PID 文件 ~/exp3_logs/sim.pid；停止用 ./stop_sim.sh
set -e
SCEN=${1:-normal}; GUI=${2:-true}; shift 2 2>/dev/null || true
WS=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
source ~/mecharm_ws/install/setup.bash
source "$WS/install/setup.bash"
mkdir -p ~/exp3_logs
LOG=~/exp3_logs/launch_${SCEN}_$(date +%Y%m%d_%H%M%S).log
[ "$GUI" = "true" ] && export DISPLAY=${DISPLAY:-:1}
setsid ros2 launch mecharm_sort sort_sim.launch.py scenario:=$SCEN gui:=$GUI "$@" > "$LOG" 2>&1 < /dev/null &
echo $! > ~/exp3_logs/sim.pid
echo "已启动 scenario=$SCEN gui=$GUI  日志：$LOG  （停止：$WS/stop_sim.sh）"
