# 【讲解】真机一键启动：USB 相机 → YOLO 识别 → 机械臂驱动（实验二 mecharm_real/real_driver，接口与仿真 ros2_control 相同）
#         → 抓取动作服务器（示教回放）→ 任务状态机（ArUco 网格定位）。
# 参数：
#   arm:=real|none        none = 不连机械臂，只跑相机 + 识别 + 网格定位（感知测试模式，看 /grid/image，Ctrl+C 结束）
#   host:=10.42.0.89      臂内 arm_server 地址
#   model:=<pt 路径>       YOLO 模型
#   taught:=<yaml 路径>    示教点文件（默认 ~/Desktop/exp3_taught_points.yaml，teach_points 工具默认写这里）
#   grid:=<yaml 路径>      网格配置（默认 config/grid.yaml，与打印的网格纸一致）
#   detector:=yolo|color  color = 用仿真的颜色分割节点（现场没有模型时应急）
#   log_dir:=~/exp3_logs_real  record:=true
#   move_duration:=2.0    每段动作秒数，越小越快（首次用 4.0，稳定后 2.0）
#   max_speed:=40         驱动速度百分比上限（首次用 20，稳定后 40；上限 100 不建议）
#   lift_deg:=12          抓起后抬多高（J2 回收度数；12°≈4 cm）   transit_lift_deg:=18  横移时的高度（18°≈5-6 cm）
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context, *args, **kwargs):
    pkg = get_package_share_directory('mecharm_sort')
    cfg = lambda n: LaunchConfiguration(n).perform(context)
    params = os.path.join(pkg, 'config', 'real.yaml')
    sm = os.path.join(pkg, 'config', 'state_machine.yaml')
    grid = cfg('grid') or os.path.join(pkg, 'config', 'grid.yaml')
    taught = cfg('taught') or os.path.expanduser('~/Desktop/exp3_taught_points.yaml')  # 工作区外，teach_points 默认写这里
    run_dir = os.path.join(os.path.expanduser(cfg('log_dir')), f'real_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    os.makedirs(run_dir, exist_ok=True)
    record = cfg('record').lower() == 'true'
    video = os.path.join(run_dir, 'camera_view.mp4') if record else ''

    nodes = [Node(package='mecharm_sort', executable='usb_camera', output='screen', parameters=[params])]
    if cfg('detector') == 'color':
        nodes.append(Node(package='mecharm_sort', executable='color_detector', output='screen',
                          parameters=[os.path.join(pkg, 'config', 'sort.yaml'), {'use_sim_time': False, 'record_video': video}]))
    else:
        nodes.append(Node(package='mecharm_sort', executable='yolo_detector', output='screen',
                          parameters=[params, {'model_path': cfg('model'), 'record_video': video}]))
    if cfg('arm') != 'real':
        # 感知测试：不连机械臂，任务节点进入 perception_only 模式（持续定位网格 + 识别，发布 /grid/image），Ctrl+C 结束
        nodes.append(Node(package='mecharm_sort', executable='task_manager', output='screen',
                          parameters=[params, {'grid_config': grid, 'state_machine_config': sm, 'log_dir': run_dir,
                                               'perception_only': True}]))
        return nodes
    # 速度：每段动作时长越短、驱动速度上限越高越快（驱动按"转多少度/几秒"折算成速度百分比，再夹在 [5, max_speed]）
    move_dur = float(cfg('move_duration'))
    max_speed = int(cfg('max_speed'))
    nodes.append(Node(package='mecharm_real', executable='real_driver', output='screen',
                      parameters=[params, {'arm_host': cfg('host'), 'max_speed': max_speed}]))
    nodes.append(Node(package='mecharm_sort', executable='pick_place_server', output='screen',
                      parameters=[params, {'grid_config': grid, 'log_dir': run_dir, 'taught_points_file': taught,
                                           'move_duration': move_dur, 'grip_duration': max(1.0, move_dur * 0.5),
                                           'lift_deg': float(cfg('lift_deg')), 'transit_lift_deg': float(cfg('transit_lift_deg'))}]))
    manager = Node(package='mecharm_sort', executable='task_manager', output='screen',
                   parameters=[params, {'grid_config': grid, 'state_machine_config': sm, 'log_dir': run_dir}])
    nodes.append(manager)
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=[Shutdown(reason='任务完成')])))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='real'),
        DeclareLaunchArgument('host', default_value='10.42.0.89'),
        DeclareLaunchArgument('model', default_value='~/Desktop/yingjiehua/best.pt'),
        DeclareLaunchArgument('taught', default_value=''),
        DeclareLaunchArgument('grid', default_value=''),
        DeclareLaunchArgument('detector', default_value='yolo'),
        DeclareLaunchArgument('log_dir', default_value='~/exp3_logs_real'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('move_duration', default_value='2.0'),   # 每段动作秒数（原 4.0）
        DeclareLaunchArgument('max_speed', default_value='40'),        # 驱动速度百分比上限（原 20）
        DeclareLaunchArgument('lift_deg', default_value='12'),         # 抓起后抬升角度（J2 回收度数，约 4 cm）
        DeclareLaunchArgument('transit_lift_deg', default_value='18'), # 横移高度角度（约 5-6 cm）
        OpaqueFunction(function=setup),
    ])
