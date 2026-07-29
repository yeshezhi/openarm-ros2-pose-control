# OpenArm ROS 2：基于 MoveIt 2 的右臂 6D 位姿控制与到达判定

> 一个基于 ROS 2 Humble、OpenArm、MoveIt 2 与 ros2_control 的仿真学习项目。  
> 系统接收外部发布的末端 6D 位姿目标，完成逆运动学、轨迹规划、控制器执行，并通过 TF 自动判定末端是否到达目标。

## 1. 项目能力

- 使用 OpenArm v2.0 的 URDF/Xacro 机器人模型。
- 在 `use_fake_hardware:=true` 假硬件环境中运行 OpenArm 双臂 MoveIt 仿真。
- 为右臂构建完整 6D 位姿目标接口：位置 `(x, y, z)` + 四元数朝向 `(qx, qy, qz, qw)`。
- 通过 MoveIt 的 `/move_action` 完成 IK 求解、轨迹规划与轨迹执行。
- 通过 `FollowJointTrajectory` 理解关节空间轨迹控制。
- 通过 TF 查询 `world → openarm_right_ee_base_link`，获取末端实时位姿。
- 自动计算目标与实际末端之间的位置误差、姿态误差，并基于容差输出任务是否完成。
- 支持不可达目标测试，理解 MoveIt 的规划失败与错误码反馈。
- 设计自定义 ExecutePose ROS 2 Action：Goal、Feedback、Result 与标准取消机制。
- 实现队列式 ExecutePose Action 服务器：最多缓存 5 个任务，支持 FIFO 调度、实时队列位置反馈，以及指定任务 ID 的精准取消。

## 2. 系统架构

```mermaid
flowchart LR
    A["外部命令行 / 上层应用"] -->|"PoseStamped"| B["/openarm/target_pose"]
    B --> C["pose_goal_moveit_client"]
    C -->|"MoveGroup Action"| D["MoveIt move_group"]
    D -->|"IK + 轨迹规划"| E["right_joint_trajectory_controller"]
    E --> F["ros2_control 假硬件"]
    F --> G["OpenArm 右臂模型 / RViz"]

    B --> H["pose_goal_reach_monitor"]
    G -->|"TF: world → 末端"| H
    H --> I["位置误差 + 姿态误差 + 到达判定"]
```

## 3. 演示截图

右臂接收 `/openarm/target_pose` 的 6D 位姿目标后，MoveIt 完成逆运动学、轨迹规划与执行；系统再通过 TF 自动计算末端位置和姿态误差，并判定是否到达目标。

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

## 6. 启动方式

### 6.1 编译学习包

```bash
# 进入 ROS 2 工作空间。
cd ~/robot_project/ros2_ws

# 加载 ROS 2 Humble。
source /opt/ros/humble/setup.bash

# 编译学习包。
colcon build --packages-select openarm_learning

# 加载当前工作空间。
source install/setup.bash
```

### 6.2 启动 OpenArm MoveIt 假硬件仿真

```bash
# 加载 ROS 2 Humble。
source /opt/ros/humble/setup.bash

# 加载当前工作空间。
source ~/robot_project/ros2_ws/install/setup.bash

# 启动 OpenArm 双臂假硬件、MoveIt 与 RViz。
ros2 launch openarm_bimanual_moveit_config demo.launch.py arm_type:=openarm_v2.0 use_fake_hardware:=true
```

### 6.3 启动位姿控制与自动验收节点

```bash
# 加载 ROS 2 Humble。
source /opt/ros/humble/setup.bash

# 加载当前工作空间。
source ~/robot_project/ros2_ws/install/setup.bash

# 启动“话题目标位姿 → MoveIt 执行 → TF 自动验收”。
ros2 launch openarm_learning topic_pose_control.launch.py
```

### 6.4 发布右臂末端 6D 位姿目标

```bash
# 加载 ROS 2 Humble。
source /opt/ros/humble/setup.bash

# 加载当前工作空间。
source ~/robot_project/ros2_ws/install/setup.bash

# 发布 world 坐标系下的完整末端目标位姿。
ros2 topic pub --once /openarm/target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.030, y: -0.154, z: 0.262}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## 7. ROS 2 接口定义

### 输入：`/openarm/target_pose`

消息类型：`geometry_msgs/msg/PoseStamped`

| 字段 | 含义 | 单位 |
|---|---|---|
| `header.frame_id` | 目标的参考坐标系 | 无，例如 `world` |
| `position.x/y/z` | 末端目标位置 | 米 |
| `orientation.x/y/z/w` | 末端目标朝向四元数 | 无单位 |

### 输出：终端日志

`pose_goal_moveit_client` 输出：

- MoveIt 是否接受目标；
- 当前规划状态；
- 最终成功或失败错误码。

`pose_goal_reach_monitor` 输出：

- 末端实际位置与目标位置的三维距离误差；
- 末端实际朝向与目标朝向的最小夹角；
- 是否满足到达条件。

## 8. 到达判定标准

当前参数：

| 指标 | 阈值 |
|---|---|
| 位置误差 | `≤ 0.005 m`，即 5 mm |
| 姿态误差 | `≤ 0.26 rad`，约 14.9°；该整体阈值与 MoveIt 三个轴各 `0.15 rad` 的约束相匹配 |

一次测试结果：

```text
目标位置：x=0.030 m，y=-0.154 m，z=0.262 m
实际位置误差：4.98 mm
实际姿态误差：6.38°
判定结果：已到达目标
```

## 9. MoveIt 规划场景与碰撞检测

项目通过 `planning_scene_obstacle.py` 调用 MoveIt 的 `/apply_planning_scene` 服务，在 `world` 坐标系中添加或删除名为 `demo_box` 的虚拟盒子障碍物。

实验采用控制变量对照：

| 条件 | 右臂末端目标 `x=0.060 m` 的结果 |
|---|---|
| 无障碍物 | MoveIt 成功规划并执行 |
| 添加并确认 `demo_box` 后 | MoveIt 无法找到可执行轨迹，机械臂保持在安全位置 |

该实验说明 MoveIt 会将规划场景中的碰撞物体纳入 IK 与轨迹规划过程，拒绝可能发生碰撞的运动请求。

## 10. 两种控制方式的区别

### 10.1 关节空间控制

通过 `right_arm_trajectory_client.py` 直接指定：

```text
joint1, joint2, joint3, joint4, joint5, joint6, joint7
```

此时开发者直接决定各关节目标角度，控制器按照给定轨迹执行。

### 10.2 末端 6D 位姿控制

通过 `/openarm/target_pose` 指定：

```text
x, y, z, qx, qy, qz, qw
```

此时由 MoveIt 自动完成：

```text
末端目标 → IK 求解关节角度 → 碰撞/约束检查 → 轨迹规划 → 控制器执行
```

## 11. 后续计划

- 支持连续目标队列与任务取消。
- 增加碰撞物体与碰撞场景规划。
- 为目标话题增加服务反馈或 Action 接口。
- 为项目增加自动化测试与持续集成。
- 接入 OpenArm 真实硬件接口，完成 CAN 通信与真实机械臂控制前的安全验证。

## 12. 简历描述参考

> 基于 ROS 2 Humble、MoveIt 2 和 ros2_control 搭建 OpenArm 双臂假硬件仿真系统；设计右臂末端 6D 位姿话题控制接口，实现 MoveIt 逆运动学求解、轨迹规划、控制器执行，以及基于 TF 的位置/姿态误差自动验收；实现 5 mm 位置容差与三个轴各 0.15 rad 姿态约束下的目标到达判定。