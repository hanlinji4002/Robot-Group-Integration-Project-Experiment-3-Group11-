#!/bin/bash
# 【讲解】停止后台仿真：按 PID 文件杀整个进程组，再清 Gazebo 残留（进程名是 ruby，按名杀不掉，用 -f 匹配命令行）。
if [ -f ~/exp3_logs/sim.pid ]; then
  PID=$(cat ~/exp3_logs/sim.pid); kill -INT -- -$PID 2>/dev/null; sleep 3; kill -9 -- -$PID 2>/dev/null; rm -f ~/exp3_logs/sim.pid
fi
pkill -9 -f 'ign gazebo.*sort_worl[d]' 2>/dev/null
pkill -9 -f 'parameter_bridg[e]' 2>/dev/null
pkill -9 -f 'mecharm_sort/(color_detecto[r]|pick_place_serve[r]|task_manage[r])' 2>/dev/null
sleep 1; echo "残留进程："; pgrep -fa 'ign gazebo|ros2 launch|mecharm_sort' | grep -v pgrep || echo 无
