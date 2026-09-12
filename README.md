# 桌面物体自动分类整理（小组实验）

> 机器人集成小组项目 Ⅰ · 实验三
>
> 仿真开发 → 真机验证。**先在仿真中打通"识别 — 判断 — 抓取 — 分类 — 异常处理"完整任务链**，再用真实相机、Jetson 和 mechArm 验证。

## 实验目标

综合使用目标检测和定点抓取：机器人自动识别桌面网格里的物体，按类别放到对应的分类区域。
本实验三类物体是 **绿 / 红 / 蓝 三色 25 mm 方块**，分类规则按方向：

| 类别 | 放到哪里 |
|---|---|
| 绿色 | 网格**左侧**空桌面（机器人左手边，+y） |
| 红色 | 网格**前方**空桌面（移出网格区，+x） |
| 蓝色 | 网格**右侧**空桌面（机器人右手边，−y） |

桌面上**没有任何盒子**，只有 6 个网格画了黑线；三个分类区就是网格外的空桌面。

## 系统接口（与实验要求一致）

| 环节 | 接口 |
|---|---|
| 图像 | `sensor_msgs/Image`（`/camera/image_raw`，仿真俯视相机经 ros_gz_bridge 桥接） |
| 检测 | `vision_msgs/Detection2DArray`（`/detections`：类别、检测框、置信度） |
| 抓取 | ROS 2 Action `mecharm_sort_interfaces/action/PickPlace`（`/pick_place`，带步骤反馈） |
| 任务控制 | 表驱动有限状态机（`config/state_machine.yaml`） |

## 仓库结构

```
.
├── README.md
├── 00-overview/
│   └── 实验3_桌面物体自动分类整理实验要求.docx
└── 01-simulation/                      仿真阶段（已完成，详见其 README）
    ├── README.md
    ├── mecharm_sort/                   ROS 2 功能包：识别、抓取动作服务器、任务状态机、模型、世界、参数、launch
    ├── mecharm_sort_interfaces/        ROS 2 接口包：PickPlace.action
    ├── scripts/                        Jetson 上的启动/停止/调试脚本
    └── results/                        验收数据：normal（6/6）、示教回放+相机换边（6/6）、abnormal（4 类异常）三次完整运行的日志、截图、相机视频
```

## 阶段二（真机）接入方式

任务节点与抓取服务器只依赖三样东西，真机阶段逐项替换、任务逻辑不改：

1. **相机**：真实俯视 RGB 相机驱动发布 `sensor_msgs/Image` 到 `/camera/image_raw` 和 `/camera/camera_info`。
2. **识别**：用前序实验训练的 YOLO 模型替换 `color_detector`，发布同样的 `vision_msgs/Detection2DArray`。
3. **机械臂**：用实验二的 `mecharm_real/real_driver` 提供 `/arm_controller`、`/hand_controller` 两个 FollowJointTrajectory 接口；
   抓取服务器参数 `grasp_check` 改为 `gripper`（按夹爪读数判夹空），任务节点 `sim_check` 改为 `false`。
4. 网格/分类区/相机外参在 `config/grid.yaml` 里按真实桌面重新量一遍即可。
