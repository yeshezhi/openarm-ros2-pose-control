# OpenArm ROS 2：基于 MoveIt 2 的右臂 6D 位姿控制、任务队列与自动验收

> 一个基于 ROS 2 Humble、OpenArm、MoveIt 2 与 ros2_control 的仿真学习项目。  
> 项目实现右臂末端 6D 位姿控制、逆运动学、轨迹规划、碰撞检测、目标到达判定，以及支持排队和精准取消的 ROS 2 Action 任务调度。

## 1. 项目能力

- 使用 OpenArm v2.0 的 URDF/Xacro 机器人模型。
- 在 `use_fake_hardware:=true` 假硬件环境中运行 OpenArm 双臂 MoveIt 仿真。
- 支持右臂末端完整 6D 位姿目标：位置 `(x, y, z)` 与四元数朝向 `(qx, qy, qz, qw)`。
- 通过 MoveIt 的 `/move_action` 完成逆运动学、轨迹规划与控制器执行。
- 通过 `FollowJointTrajectory` 理解直接指定 7 个关节角度的关节空间控制。
- 通过 TF 查询 `world → openarm_right_ee_base_link`，获取右臂末端实时位姿。
- 自动计算目标与实际末端之间的位置误差、姿态误差，并基于容差判定是否到达目标。
- 支持不可达目标与碰撞障碍物测试，理解 MoveIt 的规划失败反馈。
- 定义自定义 ROS 2 Action：`openarm_interfaces/action/ExecutePose`。
- 实现队列式 Action 服务器：
  - 最多缓存 5 条任务；
  - FIFO 顺序执行；
  - 实时反馈任务队列位置；
  - 支持取消当前任务；
  - 支持按任务 UUID 精确取消等待任务；
  - 每条任务独立返回 `SUCCEEDED`、`CANCELED` 或 `ABORTED`。

## 2. 系统架构

```mermaid
flowchart LR
    A["上层应用 / 命令行客户端"] -->|"ExecutePose Action Goal"| B["/openarm/queued_execute_pose"]
    B --> C["queued_execute_pose_action_server"]
    C -->|"FIFO 队列调度"| D["MoveIt /move_action"]
    D -->|"IK + 轨迹规划 + 碰撞检测"| E["right_joint_trajectory_controller"]
    E --> F["ros2_control 假硬件"]
    F --> G["OpenArm 右臂模型 / RViz"]

    C -->|"Action Feedback"| A
    C -->|"Action Result"| A

    G -->|"TF: world → 末端"| H["pose_goal_reach_monitor"]
    H --> I["位置误差 + 姿态误差 + 到达判定"]
```

## 3. 演示截图

右臂接收末端 6D 位姿目标后，MoveIt 完成逆运动学、轨迹规划与执行；系统再通过 TF 自动计算末端位置和姿态误差，并判定是否到达目标。

![OpenArm 右臂 6D 位姿控制与自动到达判定](docs/rviz_pose_control.png)

## 4. 软件环境

| 项目 | 当前环境 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| 机器人模型 | OpenArm v2.0 |
| 运动规划 | MoveIt 2 |
| 控制框架 | ros2_control |
| 硬件模式 | `mock_components/GenericSystem` 假硬件 |
| 可视化 | RViz 2 |
| 图形远程连接 | NoMachine |
| 代码编辑 | VS Code Remote-SSH |

> 当前项目是仿真与软件控制链路验证，尚未连接真实 CAN 总线或真实 OpenArm 硬件。

## 5. 项目目录

```text
openarm-ros2-pose-control/
├── README.md
├── docs/
│   └── rviz_pose_control.png
└── packages/
    ├── openarm_interfaces/
    │   ├── action/
    │   │   └── ExecutePose.action
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── openarm_learning/
        ├── launch/
        │   ├── right_arm_pose_demo.launch.py
        │   ├── topic_pose_control.launch.py
        │   └── queued_pose_control.launch.py
        ├── openarm_learning/
        │   ├── joint_state_monitor.py
        │   ├── end_effector_monitor.py
        │   ├── pose_publisher.py
        │   ├── right_arm_trajectory_client.py
        │   ├── moveit_position_client.py
        │   ├── moveit_pose_client.py
        │   ├── pose_goal_moveit_client.py
        │   ├── pose_goal_reach_monitor.py
        │   ├── planning_scene_obstacle.py
        │   ├── execute_pose_action_server.py
        │   └── queued_execute_pose_action_server.py
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

## 6. 编译项目

```bash
# 进入 ROS 2 工作空间。
cd ~/robot_project/ros2_ws

# 加载 ROS 2 Humble。
source /opt/ros/humble/setup.bash

# 编译本项目中的接口包与学习包。
colcon build --packages-select \
  openarm_interfaces \
  openarm_learning \
  --symlink-install

# 加载当前工作空间。
source install/setup.bash
```

验证：

```bash
# 验证自定义接口包。
ros2 pkg prefix openarm_interfaces

# 验证学习包。
ros2 pkg prefix openarm_learning

# 查看自定义 Action 定义。
ros2 interface show openarm_interfaces/action/ExecutePose
```

## 7. 启动 OpenArm MoveIt 假硬件仿真

```bash
# 加载 ROS 2 Humble。
source /opt/ros/humble/setup.bash

# 加载当前工作空间。
source ~/robot_project/ros2_ws/install/setup.bash

# 启动 OpenArm 双臂假硬件、MoveIt 与 RViz。
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  arm_type:=openarm_v2.0 \
  use_fake_hardware:=true
```

该终端需要持续运行。

## 8. 自定义 ExecutePose Action

### 8.1 Action 定义

`packages/openarm_interfaces/action/ExecutePose.action`

```text
# Goal：末端完整 6D 位姿目标。
geometry_msgs/PoseStamped target_pose
---
# Result：任务的最终结果。
bool success
int32 error_code
string message
---
# Feedback：任务执行过程中的实时状态。
uint32 queue_position
string state
```

### 8.2 Goal

调用者发送：

```text
目标参考坐标系 + 末端位置 + 末端四元数朝向
```

| 字段 | 含义 | 单位 |
|---|---|---|
| `header.frame_id` | 目标参考坐标系，例如 `world` | 无 |
| `position.x/y/z` | 末端目标位置 | 米 |
| `orientation.x/y/z/w` | 末端目标四元数朝向 | 无单位 |

### 8.3 Feedback

| 字段 | 含义 |
|---|---|
| `queue_position = 0` | 当前任务正在执行 |
| `queue_position = 1` | 当前任务是队列中第一个等待任务 |
| `queue_position = 2` | 当前任务是队列中第二个等待任务 |
| `PLANNING_AND_EXECUTING` | MoveIt 正在规划或执行 |
| `QUEUED_WAITING_FOR_TURN` | 任务正在等待前面的任务完成 |
| `CANCELING_MOVEIT_TRAJECTORY` | 正在向 MoveIt 转发取消请求 |

### 8.4 Result

| 状态 | 含义 |
|---|---|
| `SUCCEEDED` | MoveIt 已成功规划并执行 |
| `CANCELED` | 调用者取消了任务 |
| `ABORTED` | MoveIt 无法完成规划或执行 |
| `error_code = 1` | MoveIt 成功 |
| `error_code = -2` | 本项目定义：任务被调用者取消 |

## 9. 单任务 Action 控制

启动单任务 Action 服务器：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/ros2_ws/install/setup.bash

ros2 run openarm_learning execute_pose_action_server
```

发送一条右臂末端 6D 位姿任务：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/ros2_ws/install/setup.bash

ros2 action send_goal --feedback \
  /openarm/execute_pose \
  openarm_interfaces/action/ExecutePose \
  "{target_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.030, y: -0.154, z: 0.262}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

单任务 Action 服务器一次只接受一条任务；当任务正在执行时，后续任务会被拒绝。

## 10. 队列式 Action 控制

启动队列式 Action 服务器：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/ros2_ws/install/setup.bash

ros2 run openarm_learning queued_execute_pose_action_server
```

队列式服务器名称：

```text
/openarm/queued_execute_pose
```

它最多接受 5 条任务，包含当前正在执行的任务。

发送一条队列任务：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/ros2_ws/install/setup.bash

ros2 action send_goal --feedback \
  /openarm/queued_execute_pose \
  openarm_interfaces/action/ExecutePose \
  "{target_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.060, y: -0.154, z: 0.262}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

若连续发送三条任务，预期反馈如下：

```text
任务 1：queue_position = 0 → PLANNING_AND_EXECUTING
任务 2：queue_position = 1 → QUEUED_WAITING_FOR_TURN
任务 3：queue_position = 2 → QUEUED_WAITING_FOR_TURN

任务 1 完成后：
任务 2：queue_position = 0 → PLANNING_AND_EXECUTING
任务 3：queue_position = 1 → QUEUED_WAITING_FOR_TURN

任务 2 完成后：
任务 3：queue_position = 0 → PLANNING_AND_EXECUTING
```

这说明系统按照 FIFO，即先进先出顺序执行任务。

## 11. Action 任务取消

ROS 2 Humble 的 `ros2 action` 命令行工具没有 `cancel` 子命令，因此通过 Action 的隐藏取消服务完成取消。

取消服务名称：

```text
/openarm/queued_execute_pose/_action/cancel_goal
```

取消当前所有活动任务：

```bash
ros2 service call \
  /openarm/queued_execute_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
```

项目已验证：

- 当前正在执行的任务可被取消；
- 当前轨迹会停止；
- Action 返回 `CANCELED`；
- 返回 `success: false`；
- 返回项目级取消码 `error_code: -2`；
- 可以根据 Goal UUID 精确取消某一条等待任务，而不影响队列中的其他任务。

## 12. 基于 TF 的到达判定

`pose_goal_reach_monitor.py` 监听位姿目标，并通过 TF 查询：

```text
world → openarm_right_ee_base_link
```

计算：

- 目标位置与实际位置的欧氏距离；
- 目标朝向与实际朝向之间的最小夹角；
- 是否满足位置与姿态容差。

当前到达判定标准：

| 指标 | 阈值 |
|---|---|
| 位置误差 | `≤ 0.005 m`，即 5 mm |
| 姿态误差 | `≤ 0.26 rad`，约 14.9° |

姿态总误差阈值 `0.26 rad` 与 MoveIt 的三个轴分别 `0.15 rad` 的约束相匹配。

一次成功测试结果：

```text
目标位置：x=0.030 m，y=-0.154 m，z=0.262 m
实际位置误差：4.98 mm
实际姿态误差：6.38°
判定结果：已到达目标
```

## 13. MoveIt 规划场景与碰撞检测

项目通过 `planning_scene_obstacle.py` 调用 MoveIt 的：

```text
/apply_planning_scene
```

在 `world` 坐标系中添加或删除名为 `demo_box` 的虚拟盒子障碍物。

控制变量实验结果：

| 条件 | 右臂末端目标 `x=0.060 m` 的结果 |
|---|---|
| 无障碍物 | MoveIt 成功规划并执行 |
| 添加并确认 `demo_box` 后 | MoveIt 无法找到可执行轨迹，机械臂保持在安全位置 |

该实验说明 MoveIt 会将规划场景中的碰撞物体纳入逆运动学与轨迹规划过程，拒绝可能发生碰撞的运动请求。

## 14. 两种控制方式的区别

### 14.1 关节空间控制

通过 `right_arm_trajectory_client.py` 直接指定：

```text
joint1, joint2, joint3, joint4, joint5, joint6, joint7
```

开发者直接决定每个关节的目标角度，控制器按照给定关节轨迹执行。

### 14.2 末端 6D 位姿控制

通过 `ExecutePose Action` 或 `/openarm/target_pose` 指定：

```text
x, y, z, qx, qy, qz, qw
```

MoveIt 自动完成：

```text
末端目标
→ IK 求解关节角度
→ 碰撞与约束检查
→ 轨迹规划
→ ros2_control 控制器执行
```

## 15. 当前项目学习成果

- ROS 2 节点、话题、服务、TF、Launch 文件与工作空间管理。
- URDF/Xacro 机器人模型、关节链与末端坐标系。
- MoveIt 2 的逆运动学、位置目标、完整 6D 位姿目标和规划失败处理。
- ros2_control 假硬件与 `FollowJointTrajectory` 关节轨迹控制。
- MoveIt 规划场景与虚拟碰撞障碍物。
- 自定义 ROS 2 Action 接口设计。
- 多线程 Action 服务器、任务队列、反馈、Result 与取消机制。
- 基于 Git 的项目版本管理与 GitHub 远程仓库协作。

## 16. 后续计划

- 增加单元测试、集成测试与 GitHub Actions 持续集成。
- 为队列增加任务优先级、超时和重试策略。
- 增加避障路径规划，而不只是验证碰撞导致的规划失败。
- 支持动态障碍物与规划场景更新。
- 接入 OpenArm 真实硬件接口，并在真实 CAN 通信前完成安全验证。
- 增加用户图形界面或 Web 控制面板。

## 17. 简历描述参考

> 基于 ROS 2 Humble、MoveIt 2 与 ros2_control 搭建 OpenArm 双臂假硬件仿真系统；实现右臂末端 6D 位姿控制、MoveIt 逆运动学求解、轨迹规划、碰撞检测和基于 TF 的位置/姿态误差自动验收。设计自定义 ExecutePose ROS 2 Action，并实现支持最多 5 条任务的 FIFO 队列、实时队列反馈、当前任务取消与指定任务 UUID 精确取消机制。