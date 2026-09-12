# 【讲解】一键启动完整仿真系统（验收要求：一个 Launch 文件启动相机、识别、机械臂和任务控制）。
# 顺序：Gazebo 世界(含俯视相机) → 桥接(时钟/图像/内参/真值位姿) → robot_state_publisher → 生成机器人
#      → 控制器(joint_state_broadcaster → arm_controller + hand_controller) → 识别节点 + 抓取动作服务器 → 任务状态机。
# 参数：
#   scenario:=normal|abnormal   normal=6 格 6 方块全可分类；abnormal=空格/未识别/不可达/夹取失败注入
#   gui:=true|false             false 为无头（--headless-rendering，相机照常出图）
#   task:=true|false            false 只起仿真+识别+动作服务器，不跑任务（调试用）
#   log_dir:=~/exp3_logs        每次运行建子目录 <scenario>_<时间戳>，五份日志 + 扫描截图 + 相机视频都在里面
#   record:=true|false          是否把识别调试画面录成 camera_view.mp4
#   exit_on_finish:=true        任务节点退出即整体关闭（方便脚本串跑）
#   grid_mode:=aruco|camera_pose  网格定位方式：aruco = 从画面里 4 个 ArUco 码算映射（真机同款，默认）；camera_pose = 用写死的相机位姿
#   taught:=true|false          true = 抓取服务器用示教回放模式（config/taught_points_sim.yaml，由逆解生成），验证真机链路
#   camera_pose:="x y z r p y"  临时改相机位姿（覆盖世界文件里的 top_camera），用来验证相机随便摆也能工作
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SCENARIOS = {
    'normal': ('sort_world.sdf', 'grid.yaml', [0]),
    'abnormal': ('sort_world_abnormal.sdf', 'grid_abnormal.yaml', [1]),
}


def setup(context, *args, **kwargs):
    pkg = get_package_share_directory('mecharm_sort')
    scenario = LaunchConfiguration('scenario').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    task = LaunchConfiguration('task').perform(context).lower() == 'true'
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    exit_on_finish = LaunchConfiguration('exit_on_finish').perform(context).lower() == 'true'
    grid_mode = LaunchConfiguration('grid_mode').perform(context)
    taught = LaunchConfiguration('taught').perform(context).lower() == 'true'
    camera_pose = LaunchConfiguration('camera_pose').perform(context).strip()
    if scenario not in SCENARIOS:
        raise RuntimeError(f'scenario 只能是 {list(SCENARIOS)}')
    world_file, grid_file, inject = SCENARIOS[scenario]
    world = os.path.join(pkg, 'worlds', world_file)
    grid = os.path.join(pkg, 'config', grid_file)
    xacro_file = os.path.join(pkg, 'urdf', 'arm_model.xacro')
    controllers = os.path.join(pkg, 'config', 'controllers.yaml')
    params = os.path.join(pkg, 'config', 'sort.yaml')
    sm = os.path.join(pkg, 'config', 'state_machine.yaml')
    run_dir = os.path.join(os.path.expanduser(LaunchConfiguration('log_dir').perform(context)),
                           f'{scenario}_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    os.makedirs(run_dir, exist_ok=True)
    if camera_pose:  # 把世界文件里 top_camera 的 pose 换掉，写到本次运行目录再启动
        import re
        src = open(world, encoding='utf-8').read()
        src, n = re.subn(r'(<model name="top_camera">\s*<static>true</static>\s*<pose>)[^<]+(</pose>)',
                         lambda m: m.group(1) + camera_pose + m.group(2), src, count=1)
        if n != 1:
            raise RuntimeError('世界文件里找不到 top_camera 的 pose')
        world = os.path.join(run_dir, world_file)
        open(world, 'w', encoding='utf-8').write(src)
    with open(os.path.join(run_dir, 'launch_args.txt'), 'w') as f:
        f.write(f'scenario={scenario}\nworld={world}\ngrid={grid}\ngui={gui}\nrecord={record}\n'
                f'grid_mode={grid_mode}\ntaught={taught}\ncamera_pose={camera_pose}\n')

    desc_pkg = get_package_share_directory('mycobot_description')
    share_roots = [os.path.dirname(pkg), os.path.dirname(desc_pkg)]
    resource_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        ':'.join(share_roots + [os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]))

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' controllers_file:=', controllers]), value_type=str)

    if gui:
        gz = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world], output='screen')
    else:
        gz = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', world], output='screen')

    # 桥接：时钟、相机图像与内参（GZ→ROS）、所有会动的实体位姿真值（GZ→ROS，仿真判定用）
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/world/sort_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '--ros-args', '-r', '/world/sort_world/dynamic_pose/info:=/sim/poses',
        ],
        output='screen')

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'mecharm', '-x', '0', '-y', '0', '-z', '0.75'],
                 output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager-timeout', '90'], output='screen')

    jsb, arm, hand = spawner('joint_state_broadcaster'), spawner('arm_controller'), spawner('hand_controller')

    detector = Node(
        package='mecharm_sort', executable='color_detector', output='screen',
        parameters=[params, {'record_video': os.path.join(run_dir, 'camera_view.mp4') if record else ''}])
    server_over = {'grid_config': grid, 'log_dir': run_dir, 'inject_grasp_fail_grids': inject}
    if taught:
        server_over.update({'use_taught_joints': True,
                            'taught_points_file': os.path.join(pkg, 'config', 'taught_points_sim.yaml')})
    server = Node(
        package='mecharm_sort', executable='pick_place_server', output='screen',
        parameters=[params, server_over])
    manager = Node(
        package='mecharm_sort', executable='task_manager', output='screen',
        parameters=[params, {'grid_config': grid, 'state_machine_config': sm, 'log_dir': run_dir, 'grid_mode': grid_mode}])

    actions = [resource_env, gz, bridge, rsp, spawn,
               RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
               RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, hand])),
               RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[detector, server])),
               ]
    if task:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=hand, on_exit=[manager])))
        if exit_on_finish:
            actions.append(RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=[Shutdown(reason='任务完成')])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='normal'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('task', default_value='true'),
        DeclareLaunchArgument('log_dir', default_value='~/exp3_logs'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('exit_on_finish', default_value='true'),
        DeclareLaunchArgument('grid_mode', default_value='aruco'),
        DeclareLaunchArgument('taught', default_value='false'),
        DeclareLaunchArgument('camera_pose', default_value=''),
        OpaqueFunction(function=setup),
    ])
