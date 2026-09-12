# 【讲解】真机一键启动：先起 real_driver（连臂内服务），4 s 后起任务节点 grasp_task（与仿真同一份程序，只换参数文件）。
# 任务节点退出时整个 launch 一起退出，退出码 0，所以可以用 && 把多次运行串起来。参数：cycles 次数、host 臂内地址、log_dir 日志目录。
"""真机（Jetson + mechArm 270）定点抓取一键启动。

启动：真机驱动（mecharm_real/real_driver，经 TCP 调臂内 arm_server.py）
    + 任务节点（mecharm_grasp/grasp_task，与仿真同一份代码）。
任务节点跑完自动退出时整个 launch 一起退出（退出码 0），便于用 && 串多次运行。
用法：
  ros2 launch mecharm_real real.launch.py                          # 5 次抓取
  ros2 launch mecharm_real real.launch.py cycles:=1                # 先单次低速验证
  ros2 launch mecharm_real real.launch.py cycles:=4 log_dir:=~/grasp_logs_real/run1
  ros2 launch mecharm_real real.launch.py host:=10.42.0.89         # 指定臂内服务地址
  软件急停（另开终端）：ros2 topic pub --once /soft_stop std_msgs/msg/Bool "data: true"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('mecharm_real'), 'config', 'real.yaml')
    cycles = LaunchConfiguration('cycles')
    host = LaunchConfiguration('host')
    log_dir = LaunchConfiguration('log_dir')

    driver = Node(package='mecharm_real', executable='real_driver', output='screen',
                  parameters=[params, {'arm_host': ParameterValue(host, value_type=str)}])
    task = Node(package='mecharm_grasp', executable='grasp_task', output='screen',
                parameters=[params, {'cycles': ParameterValue(cycles, value_type=int),
                                     'log_dir': ParameterValue(log_dir, value_type=str)}])

    return LaunchDescription([
        DeclareLaunchArgument('cycles', default_value='5'),
        DeclareLaunchArgument('host', default_value='10.42.0.89'),
        DeclareLaunchArgument('log_dir', default_value='~/grasp_logs_real'),
        driver,
        # 驱动先连上臂内服务，任务节点延后启动
        TimerAction(period=4.0, actions=[task]),
        # 任务节点退出 → 关掉整个 launch（驱动一起停），命令行才会返回
        RegisterEventHandler(OnProcessExit(target_action=task, on_exit=[Shutdown()])),
    ])
