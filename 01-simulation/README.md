# 01-simulation 仿真阶段

这是一个**机械臂自动分拣的仿真程序**：桌上 6 个网格里放着红/绿/蓝三色小方块，一台俯视相机看着桌面，
程序自己判断每个格子里是什么颜色、在哪一格，然后让虚拟机械臂把绿的放到左边、红的放到前面、蓝的放到右边，
一格一格做完为止；碰到空格、认不出的东西、够不着的格子、没夹住的情况，它会自己跳过或重试，不需要人插手。

程序装在 **Jetson** 里，你用 **Mac** 连过去下命令（和实验二一样）。

---

## 一、文件结构

```
01-simulation/
├── README.md
├── mecharm_sort_interfaces/            ROS 2 接口包（CMake）
│   ├── package.xml / CMakeLists.txt
│   └── action/PickPlace.action         抓取动作：command / grid_id / zone_id / slot → success / failed_stage / message；反馈 stage
├── mecharm_sort/                       ROS 2 功能包（纯 Python，整个目录拷进工作区 src 即可编译；仿真与真机共用）
│   ├── package.xml / setup.py / setup.cfg / resource/mecharm_sort
│   ├── mecharm_sort/
│   │   ├── color_detector.py           【识别·仿真】Image → HSV 颜色分割 → Detection2DArray（类别/框/置信度）+ 调试图 + 可选录像
│   │   ├── yolo_detector.py            【识别·真机】同样的输入输出，换成 YOLOv8 推理（best.pt，CUDA FP16）
│   │   ├── usb_camera.py               【相机·真机】OpenCV 读 /dev/video0 → sensor_msgs/Image（只保留最新帧，防积压）
│   │   ├── grid_locator.py             【定位】画面四角 ArUco 码 → 单应矩阵 → 像素落格（含视差修正），仿真真机同一套
│   │   ├── pick_place_server.py        【抓取】PickPlace 动作服务器：按网格编号查固定点、执行 14 步取放、夹取校验、失败安全返回
│   │   ├── task_manager.py             【任务】表驱动状态机：扫描 → 像素→网格映射 → 选目标 → 调动作 → 异常分流 → 日志/汇总
│   │   ├── planner.py                  固定点规划：网格/区域 → 全部路径点关节角（服务器与离线校验共用）
│   │   ├── kinematics.py               mechArm 270 正/逆运动学、限位、虎口对齐、夹爪关键点
│   │   ├── teach_points.py             【示教·真机】record / apply / show / goto，记录六关节角并自动低速回放验证（含限位检查）
│   │   └── gz_truth.py                 仿真真值订阅（只用于成功判定与夹取校验，真机没有）
│   ├── model/
│   │   ├── arm_model.xacro             机械臂模型（沿用实验二：惯量、限位、夹爪 mimic、碰撞盒）
│   │   ├── gen_world.py                世界生成器：从 grid.yaml 生成 SDF（网格黑线、方块、侧上方俯视相机；没有盒子）
│   │   ├── sort_world.sdf              正常场景：6 格各一个方块（绿红蓝各 2）
│   │   └── sort_world_abnormal.sdf     异常场景：1 绿(注入夹取失败) 2 黄(未识别) 3 蓝 4 空 5 红 6 红(格子挪到不可达处)
│   ├── config/
│   │   ├── grid.yaml                   【桌面网格配置】3×2 连片网格（格 5×6 cm，行 x=0.105/0.155）、分类区域、ArUco 码坐标、方块尺寸
│   │   ├── grid_abnormal.yaml          同上，6 号格挪到 x=0.26（不可达）
│   │   ├── state_machine.yaml          【状态机配置文件】状态、转移、重试次数、扫描帧数、置信度阈值
│   │   ├── sort.yaml                   仿真：三个节点的参数（颜色阈值、运动参数、类别→区域表）
│   │   ├── real.yaml                   真机：相机设备、YOLO 权重、示教回放、抬升/横移距离、速度等
│   │   ├── taught_points.yaml          真机示教点（6 个取物点 + 各区放置点）
│   │   ├── taught_points_sim.yaml      仿真里由逆解生成的示教点，用于验证回放链路
│   │   ├── controllers.yaml            ros2_control 控制器（沿用实验二）
│   │   ├── grid_sheet_A4.pdf / .png    【网格纸】A4 横向 100% 打印，3×2 网格 + 四角 ArUco 码
│   │   ├── make_markers.py             生成 markers/ 下的 4 张 ArUco 码
│   │   ├── make_taught_points.py       由逆解批量生成示教点文件（仿真验证用）
│   │   └── check_layout.py             离线校验：全部"网格×区域×槽位"逆解 + 路径最低点检查，改布局先跑它
│   ├── markers/
│   │   └── aruco_0..3.png              网格纸四角的定位码（DICT_4X4_50，边长 3.5 cm）
│   └── launch/
│       ├── sort_sim.launch.py          【仿真】Gazebo+相机 → 桥接 → 机器人+控制器 → 识别 → 抓取服务器 → 任务节点
│       └── sort_real.launch.py         【真机】usb_camera → yolo_detector → 抓取服务器 → 任务节点 → real_driver
├── scripts/
│   ├── run_sim.sh / stop_sim.sh        Jetson 上后台启动/停止
│   └── probe_poses.py                  调试：把 Gazebo 里夹爪/方块真实位姿记成 CSV，对照运动学模型
├── 仿真实验报告.md
└── results/
    ├── normal/                         正常场景完整一轮：6/6 分类正确
    ├── normal_taught_camera_right/     示教回放模式 + 相机换到右侧：6/6 分类正确
    ├── abnormal/                       异常场景完整一轮：空格、未识别、不可达、夹取失败四类异常全部正确处理
    └── yolo_sim_frame.png              真机 YOLO 节点直接跑在仿真画面上的截图（接口打通验证）
```

---

## 二、使用说明（Jetson）

### 1. 连接与环境

Mac 用 USB-C 线连 Jetson，终端里：

```bash
ssh nvidia@192.168.55.1
```

每开一个新终端先加载环境（下面所有命令的前提）：

```bash
source /opt/ros/humble/setup.bash && source ~/mecharm_ws/install/setup.bash && source ~/Desktop/exp3_sim_ws/install/setup.bash
```

### 2. 编译（代码有改动时）

把 `mecharm_sort` 和 `mecharm_sort_interfaces` 两个目录放进 `~/Desktop/exp3_sim_ws/src/`，然后：

```bash
cd ~/Desktop/exp3_sim_ws && colcon build --symlink-install --packages-select mecharm_sort_interfaces mecharm_sort
```

底层依赖来自实验二已装好的 `~/mecharm_ws`（厂商模型 `mycobot_description`、`gz_ros2_control`），系统里的 `ros-humble-vision-msgs`、`cv-bridge`、`ros-gz-*` 都已预装。

### 3. 运行

无头运行（不开画面，最稳，验收数据都是这样跑的）：

```bash
ros2 launch mecharm_sort sort_sim.launch.py gui:=false
```

带 Gazebo 画面（画面出现在 Jetson 显示器上）：

```bash
export DISPLAY=:1 && ros2 launch mecharm_sort sort_sim.launch.py
```

异常测试场景：

```bash
ros2 launch mecharm_sort sort_sim.launch.py gui:=false scenario:=abnormal
```

也可以用脚本后台跑：`~/Desktop/exp3_sim_ws/run_sim.sh normal false`，停止用 `stop_sim.sh`。

launch 参数：`scenario:=normal|abnormal`、`gui:=true|false`、`task:=false`（只起仿真不跑任务，调试用）、
`log_dir:=~/exp3_logs`、`record:=false`（不录相机视频）、`exit_on_finish:=false`（任务结束后不关仿真）。

任务会自己跑完并退出。**任务开始后不需要、也不能人工指定任何类别、网格或放置区**。

### 4. 看结果

每次运行在 `~/exp3_logs/<scenario>_<时间>/` 下生成：

| 文件 | 内容 |
|---|---|
| `summary.txt` | 汇总：分类成功个数、各区域数量、跳过的网格、异常统计 |
| `task_state.log` | 任务状态日志：每次状态转移、扫描结果、每步动作反馈 |
| `detections.csv` | 识别日志：每次扫描每格的类别、置信度、投票帧数、像素坐标、换算的世界坐标 |
| `grasp_results.csv` | 抓取日志：每次动作的网格、类别、区域、槽位、尝试次数、成败、失败阶段、耗时、落点核对 |
| `errors.log` | 异常日志：空格 / 未识别 / 不可达 / 夹取失败 / 通信故障 |
| `trajectory.csv` | 关节轨迹（/joint_states 每帧） |
| `scan_NN.png` | 每次扫描时的识别画面（带检测框） |
| `camera_view.mp4` | 整个过程的相机视角视频（带检测框） |

```bash
cat ~/exp3_logs/normal_*/summary.txt
```

---

## 三、它是怎么工作的

```
Gazebo 俯视相机 ──Image──▶ color_detector ──Detection2DArray──▶ task_manager ──PickPlace.action──▶ pick_place_server ──FollowJointTrajectory──▶ ros2_control/Gazebo
                                                                    ▲                                                    │
                                                                    └──────────── 结果 / 反馈 ◀───────────────────────────┘
```

1. **识别**（`color_detector`）：把图像转 HSV，取"饱和度高且够亮"的像素做前景（灰桌面、白机械臂都不饱和），
   连通域分块，每块按色相投票判红/绿/蓝；投票纯度不够（黄色方块）就标 `unknown`、低置信度。
   真机阶段换成 YOLO 节点，话题和消息不变。
2. **网格判断**（`task_manager`）：检测框中心像素 → 用相机内参（`camera_info`）和外参（`grid.yaml`）算出射线 → 与方块顶面平面相交
   得到桌面坐标 → 离哪个网格中心最近（容差 2.5 cm）就算在哪格。连收 5 帧投票，抗抖动；只在回零姿态下做识别。
   **像素怎么变成桌面坐标**（`grid_locator.py`）：网格纸四角印了 4 个 ArUco 码，它们的桌面坐标写在 `grid.yaml`；
   每次扫描先在画面里找码，用码中心解一个单应矩阵（像素 → 桌面平面），检测框中心乘上去就是桌面坐标。
   所以**相机不需要固定也不需要标定**，每次重摆都行，只要看得见网格和至少 3 个码；找不到码会等一下重扫，多次失败才安全停止。
   单应矩阵只对桌面上的点准确，方块顶面高 2.5 cm，相机斜着看会投影到更远处（相机越低偏得越多）；程序用单应矩阵反推相机位置，
   再把检测点沿朝相机方向拉回（视差修正），真机低角度相机下偏差从 3-4 cm 降到 1 cm 内。
   仿真里也用同一套：世界里贴了 4 个码贴图，`camera_pose:=` 把相机挪到别处照样能跑。
   相机建议装在桌面**左侧上方**斜向下看：机械臂回零姿态是大臂竖直、小臂水平向前，正上方看时小臂会挡住中间一列。
3. **选目标**：按网格编号从小到大，第一个"类别已知、置信度 ≥ 0.5、没被跳过"的格子；类别→区域查表（绿→left，红→front，蓝→right）；
   同一区域第 k 个物体放第 k 个槽位（间距 3.5 cm，不叠放）。
4. **抓取**（`pick_place_server`）：网格编号 → 固定取物点（不做任意位置抓取）。两种来源：
   仿真用逆解（启动时把 6 格 + 8 个落点全部预算好）；真机用**示教回放**（`use_taught_joints: true`，
   `taught_points.yaml` 里每格一组取物关节角、每个放置槽位一组关节角，上方点 = J2 往回收 25°，不做逆解）。
   示教回放链路已在 Gazebo 里用逆解生成的 taught_points_sim.yaml 验证过。
   14 步：张爪 → 取物点上方 → 下降 → 夹取 → 抬升 → **夹取校验** → 过渡高度 → 转向区域 → 放置点上方 → 下降 → 释放 → 抬升 → 转回正前 → **回零**。
   每步先查关节限位再下发。夹着方块的全部路径点工具都保持竖直（倾角 ≤ 15°），放置时降到方块底离桌面几毫米才松爪。
   回零 = 六关节全零（大臂竖直、小臂水平向前），与实验二和真机一致。

**桌面布局**（`grid.yaml`，机械臂底座在 (0, 0)，x 向前、y 向左，单位 m）：取物网格是 3 列 × 2 行连成一片的大网格，
格子 5 cm(x) × 6 cm(y)，两行中心 x = 0.105 / 0.155，三列 y = +0.06 / 0 / −0.06，物体放在格子中心。
分类区域没有盒子：绿 → 左侧空区 (0.095, +0.14)，蓝 → 右侧空区 (0.095, −0.14)，红 → 前方空区 (0.20, 0)。
网格已经推到 mechArm 270 竖直抓取的可达前沿（远行外侧格半径 0.166 m，前方区 0.20 m 是极限，所以前方区只留 2 个落点）；
再往前机械臂就够不到了，`config/check_layout.py` 里有整张可达表。
5. **循环**：每放完一个就回扫描姿态重新识别，直到所有网格为空或剩下的都是异常格。
6. **状态机**（`state_machine.yaml`）：`INIT → PARK → SCAN → SELECT → PICK_PLACE → SCAN …`，失败进 `HANDLE_FAIL` 分流。

### 异常处理

| 异常 | 怎么发现 | 怎么处理 | 日志 |
|---|---|---|---|
| 空网格 | 扫描时该格没有检测 | 跳过 | `errors.log [empty_grid]` |
| 未识别物体 | 类别 `unknown` 或置信度 < 0.5 | 跳过该格 | `[unknown_object]` |
| 目标不可达 / 关节超限 | 规划阶段无逆解或超限（动作不执行） | 跳过该格，机械臂留在回零姿态 | `[unreachable]` |
| 抓取失败（夹空） | 抬升后物体没跟着起来（仿真真值 / 真机夹爪读数） | 松爪回零，重试 1 次，再失败放弃该格 | `[grasp_failed]` |
| 轨迹执行失败（被顶住等） | 控制器超时/拒绝 | 若手里有物体先回放置点上方释放，再回零；按抓取失败处理 | `[trajectory_failed]` |
| 通信故障 | 动作服务器无响应 | 安全停止（ABORT） | `[comm]` |

---

## 四、验收数据（results/）

### 正常场景 `results/normal/`（6 个物体，3 类）

| 轮 | 网格 | 类别 | 区域 | 结果 | 落点核对 |
|---|---|---|---|---|---|
| 1 | 1 | green | left | 成功 | 通过 |
| 2 | 2 | red | front | 成功 | 通过 |
| 3 | 3 | blue | right | 成功 | 通过 |
| 4 | 4 | blue | right | 成功 | 通过 |
| 5 | 5 | green | left | 成功 | 通过 |
| 6 | 6 | red | front | 成功 | 通过 |

**6/6 正确整理**（要求 ≥ 5/6），全程无碰撞、无超限，每轮约 32 s；搬运全程工具竖直，贴近桌面才松爪。

### 异常场景 `results/abnormal/`

| 网格 | 布置 | 系统行为 |
|---|---|---|
| 1 | 绿方块，注入第 1 次夹取"不合爪" | 夹取校验发现"抬升 0 mm"→ 重试 → 第 2 次成功 |
| 2 | 黄方块 | 识别为 `unknown 0.20` → 记未识别，跳过 |
| 3 | 蓝方块 | 正常分类到 right |
| 4 | 空 | 记空网格，跳过 |
| 5 | 红方块 | 正常分类到 front |
| 6 | 红方块，格子挪到 x=0.26 | 规划无逆解 → 记不可达，跳过，机械臂不动 |

4 类异常（空网格、未识别、不可达、夹取失败）全部正确处理（要求 ≥ 2 类），任务正常结束。

---

## 五、改布局怎么办

网格、分类区、相机位置都只在 `config/grid.yaml` 里改，然后：

```bash
python3 config/check_layout.py config/grid.yaml      # 逆解可达 + 路径最低点全部通过再继续
python3 model/gen_world.py ../config/grid.yaml normal sort_world.sdf   # 在 model/ 目录下执行，重新生成世界
```

`check_layout.py` 用的是和抓取服务器同一个规划器，它说通过，仿真里就不会撞。

---

## 六、常见问题

- **相机画面里有机械臂挡住网格**：只在回零姿态下才做识别投票，机械臂运动中的检测结果不用；相机装在左侧上方就是为了回零时不挡网格。
- **Jetson 无头模式相机没图**：launch 用了 `--headless-rendering`，日志里 `libEGL warning: nvidia-drm` 是无害警告。
- **第一次动作前等十几秒**：抓取服务器启动时在预算全部路径点（Jetson 上约 10 s），算完才上线。
- **日志里 `grid_not_found`**：画面里找不到 3 个以上的 ArUco 码。检查相机是否对准网格、码有没有被方块/手臂挡住、光线是否过曝。
- **真机怎么跑**：见 `../02-real-robot/README.md`（打印网格纸 → 摆相机 → 示教 → `sort_real.launch.py`）。
- **改了 `grid.yaml` 忘了重新生成世界**：方块位置和网格配置对不上，识别到的格子会错位，一定要跑 `gen_world.py`。
- **停不干净**：`~/Desktop/exp3_sim_ws/stop_sim.sh`，Gazebo 进程名是 ruby，按名杀不掉，脚本里用命令行匹配。
