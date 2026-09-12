# 02-real-robot 真机阶段

真机阶段**复用实验二的机械臂驱动**，任务逻辑（识别 → 网格判断 → PickPlace 动作 → 状态机）与仿真是同一份代码，
启动文件在 `../01-simulation/mecharm_sort/launch/sort_real.launch.py`。

```
02-real-robot/
├── README.md
├── mecharm_real/        实验二的 Jetson 侧驱动包（原样复制，未改动）：real_driver 对外提供与仿真 ros2_control 一模一样的
│                        /arm_controller、/hand_controller 两个 FollowJointTrajectory 动作 + /joint_states，内部走网口到臂内 arm_server
│                        （含 teach.py：读当前六关节角，示教用）
└── arm_pi/              臂内树莓派上的 arm_server.py / arm_common.py（实验二原样），pymycobot 控制 mechArm 270
```

## 真机与仿真的差别（全部是参数和外设，逻辑不变）

| 项 | 仿真 | 真机 |
|---|---|---|
| 相机 | Gazebo 相机 | USB 相机（Realtek 0bda:3035，1280×720），节点 `usb_camera` |
| 识别 | 颜色分割 `color_detector` | YOLO `yolo_detector`，模型 `~/Desktop/yingjiehua/best.pt`（blue/green/red/yellow_cube）；可换 TensorRT 引擎 `best.engine`（`model:=` 参数） |
| 网格定位 | ArUco（与真机相同） | ArUco：网格纸四角 4 个码，相机随便摆，每次扫描自动算映射 |
| 抓取点 | 逆解 | 示教回放 `~/Desktop/exp3_taught_points.yaml`（6 个取物点 + 3 个空中放置点，上方点 = J2 抬 25°） |
| 机械臂 | gz_ros2_control | `mecharm_real/real_driver` → 臂内 `arm_server` |
| 夹取校验 | Gazebo 真值 | 驱动读夹爪值（夹空/夹偏时合爪动作直接失败 → 任务节点重试） |
| 落点核对 | Gazebo 真值 | 现场人工核对 |

## 现场步骤

1. **打印网格纸**：`01-simulation/mecharm_sort/config/grid_sheet_A4.pdf`，A4 横向 100% 打印（不要缩放）。
   纸的近边中点对准机器人正前方，近边距底座中心 4 cm，用胶带固定。网格 3×2、格 5×6 cm、四角 4 个 ArUco 码。
2. **摆相机**：任意位置，只要画面里能看到网格和至少 3 个码，机械臂回零姿态不挡网格（建议放网格左侧上方斜看）。
   相机可以每次重摆，不需要标定。
3. **示教**：`ros2 run mecharm_sort teach_points all`（记录后自动低速回放验证） —— 6 个格的取物关节角（夹爪张开、指垫在方块中部高度）
   + 左/前/右各 1 个空中放置点（到点直接松爪扔下，人随手拿走）。文件在 `~/Desktop/exp3_taught_points.yaml`。
4. **编译**：把 `mecharm_real` 和 `mecharm_sort`、`mecharm_sort_interfaces` 一起放进工作区 `src/` 编译。
5. **臂内**：树莓派上启动 `arm_server.py`（实验二流程），Jetson 能 ping 通臂（默认 10.42.0.89）。
6. **先只测感知**：`ros2 launch mecharm_sort sort_real.launch.py arm:=none`，看 `/detections/image`
   里是否画出 4 个码、6 个格心十字和方块框。
7. **正式运行**：`ros2 launch mecharm_sort sort_real.launch.py host:=<臂 IP>`。日志在 `~/exp3_logs_real/real_<时间>/`。
