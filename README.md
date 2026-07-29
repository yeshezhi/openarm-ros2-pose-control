# OpenArm ROS 2：基于 MoveIt 2 的右臂 6D 位姿控制、任务队列与自动验收

> 一个基于 ROS 2 Humble、OpenArm、MoveIt 2 与 ros2_control 的机械臂仿真控制项目。  
> 项目实现了右臂末端 6D 位姿控制、MoveIt 逆运动学与轨迹规划、碰撞场景管理、TF 闭环到达判定，以及支持 FIFO 队列和取消操作的自定义 ROS 2 Action 接口。

![OpenArm 右臂 6D 位姿控制与到达判定](docs/rviz_pose_control.png)

---

## 1. 项目能力

- 使用 OpenArm v2.0 的 URDF/Xacro 模型搭建双臂仿真环境。
- 在 `use_fake_hardware:=true` 的 `mock_components/GenericSystem` 假硬件环境中运行。
- 支持右臂末端完整 6D 位姿目标：位置 `(x, y, z)` 与四元数朝向 `(qx, qy, qz, qw)`。
- 通过 MoveIt 2 自动完成逆运动学、碰撞检测、轨迹规划与控制器执行。
- 支持直接指定 7 个关节角度的关节空间轨迹控制。
- 支持指定末端 6D 位姿、由 MoveIt 自动求解关节轨迹。
- 通过 TF 查询 `world → openarm_right_ee_base_link`，获取末端实时位姿。
- 自动计算目标与实际末端之间的位置误差、姿态误差，并判断是否到达。
- 定义自定义 ROS 2 Action：`openarm_interfaces/action/ExecutePose`。
- 支持单任务执行、最多 5 条任务的 FIFO 队列、实时反馈和执行中取消。
- 支持向 MoveIt Planning Scene 添加、查询和删除虚拟碰撞盒。
- 已通过 GitHub Actions 完成基础 ROS 2 构建与接口检查。

---

## 2. 系统架构

```mermaid
flowchart LR
    A["上层应用 / ROS 2 Action 客户端"] -->|"ExecutePose Goal"| B["/openarm/queued_execute_pose"]
    B --> C["queued_execute_pose_action_server"]
    C -->|"FIFO 任务调度"| D["MoveIt /move_action"]
    D -->|"IK + 碰撞检测 + 轨迹规划"| E["right_joint_trajectory_controller"]
    E --> F["ros2_control 假硬件"]
    F --> G["OpenArm 右臂模型 / RViz"]

    C -->|"Feedback / Result"| A

    G -->|"TF: world → openarm_right_ee_base_link"| H["pose_goal_reach_monitor"]
    H --> I["位置误差 + 姿态误差 + 到达判定"]
```

控制链路：

```text
6D 位姿目标
    ↓
自定义 ExecutePose Action
    ↓
任务队列 / FIFO 调度 / 取消处理
    ↓
MoveIt 逆运动学、碰撞检测、轨迹规划
    ↓
ros2_control 轨迹控制器
    ↓
OpenArm 假硬件与 RViz 仿真
    ↓
TF 获取真实末端位姿
    ↓
位置与姿态误差闭环验收
```

---

## 3. 软件环境

| 项目 | 当前环境 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| 机器人模型 | OpenArm v2.0 |
| 运动规划 | MoveIt 2 |
| 控制框架 | ros2_control |
| 硬件模式 | `mock_components/GenericSystem` 假硬件 |
| 可视化 | RViz 2 |
| 远程图形桌面 | NoMachine |
| 代码编辑 | VS Code Remote-SSH |
| Python | Python 3.10 |

> 当前项目仅验证软件控制链路与假硬件仿真，未连接真实 CAN 总线或真实 OpenArm 机械臂。

---

## 4. 仓库结构

```text
openarm-ros2-pose-control/
├── .github/
│   └── workflows/
│       └── ros2-ci.yml
├── docs/
│   └── rviz_pose_control.png
├── packages/
│   ├── openarm_interfaces/
│   │   ├── action/
│   │   │   └── ExecutePose.action
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── openarm_learning/
│       ├── launch/
│       │   ├── right_arm_pose_demo.launch.py
│       │   ├── topic_pose_control.launch.py
│       │   └── queued_pose_control.launch.py
│       ├── openarm_learning/
│       │   ├── joint_state_monitor.py
│       │   ├── end_effector_monitor.py
│       │   ├── right_arm_trajectory_client.py
│       │   ├── moveit_position_client.py
│       │   ├── moveit_pose_client.py
│       │   ├── pose_goal_moveit_client.py
│       │   ├── pose_goal_reach_monitor.py
│       │   ├── planning_scene_obstacle.py
│       │   ├── execute_pose_action_server.py
│       │   └── queued_execute_pose_action_server.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── .gitignore
└── README.md
```

---

## 5. 编译项目

```bash
# 进入 ROS 2 工作空间。
cd ~/robot_project/ros2_ws

# 加载 ROS 2 Humble。
source /opt/ros/humble/setup.bash

# 编译自定义 Action 接口包和学习控制包。
colcon build --packages-select \
  openarm_interfaces \
  openarm_learning \
  --symlink-install

# 加载编译结果。
source install/setup.bash
```

验证软件包：

```bash
ros2 pkg prefix openarm_interfaces
ros2 pkg prefix openarm_learning
ros2 interface show openarm_interfaces/action/ExecutePose
```

---

## 6. 启动 OpenArm MoveIt 假硬件仿真

```bash
cd ~/robot_project/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch openarm_bimanual_moveit_config demo.launch.py \
  arm_type:=openarm_v2.0 \
  use_fake_hardware:=true
```

启动后，可在 RViz 中选择 `right_arm` 规划组，观察右臂的规划与执行效果。

---

## 7. 两种控制方式

### 7.1 关节空间控制

关节空间控制直接指定右臂 7 个关节的目标角度：

```text
joint1, joint2, joint3, joint4, joint5, joint6, joint7
```

控制链路：

```text
目标关节角度
    ↓
FollowJointTrajectory
    ↓
right_joint_trajectory_controller
    ↓
ros2_control 假硬件
```

对应节点：

```text
right_arm_trajectory_client.py
```

适用于已经知道各关节目标角度的场景。

### 7.2 末端 6D 位姿控制

末端控制指定机械臂末端的目标位置和朝向：

```text
x, y, z, qx, qy, qz, qw
```

MoveIt 自动完成：

```text
末端目标位姿
    ↓
逆运动学 IK：求解 7 个关节角度
    ↓
碰撞检测与约束检查
    ↓
轨迹规划
    ↓
控制器执行
```

对应节点：

```text
moveit_position_client.py
moveit_pose_client.py
pose_goal_moveit_client.py
```

这种方式更符合实际机器人应用：上层程序描述“机械臂末端要去哪里”，MoveIt 决定“各关节如何运动”。

---

## 8. 自定义 Action 接口

项目定义了完整的 ROS 2 Action：

```text
openarm_interfaces/action/ExecutePose
```

接口定义：

```text
# Goal：调用者发送的完整末端 6D 位姿目标。
geometry_msgs/PoseStamped target_pose
---
# Result：任务结束后返回的结果。
bool success
int32 error_code
string message
---
# Feedback：执行过程中的实时反馈。
uint32 queue_position
string state
```

字段说明：

| 字段 | 含义 |
|---|---|
| `target_pose` | 目标末端完整 6D 位姿 |
| `success` | 是否成功完成 |
| `error_code` | MoveIt 或任务执行错误码 |
| `message` | 结果说明 |
| `queue_position` | 当前任务在队列中的位置 |
| `state` | 当前任务状态 |

---

## 9. 单个 6D 位姿任务执行

启动普通 Action 服务端：

```bash
cd ~/robot_project/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run openarm_learning execute_pose_action_server
```

新开终端，发送目标：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/ros2_ws/install/setup.bash

ros2 action send_goal --feedback \
  /openarm/execute_pose \
  openarm_interfaces/action/ExecutePose \
  "{target_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.030, y: -0.154, z: 0.262}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

正常反馈示例：

```text
state: BUILDING_MOVEIT_GOAL
state: WAITING_MOVEIT_ACCEPTANCE
state: PLANNING_AND_EXECUTING

Result:
  success: true
  error_code: 1
  message: MoveIt 已成功规划并执行 6D 位姿任务

Goal finished with status: SUCCEEDED
```

---

## 10. FIFO 任务队列

项目实现了最多容纳 5 条任务的 FIFO 队列 Action 服务端。

启动队列服务端：

```bash
cd ~/robot_project/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run openarm_learning queued_execute_pose_action_server
```

队列 Action 名称：

```text
/openarm/queued_execute_pose
```

后进入队列的任务会收到：

```text
queue_position: 1
state: QUEUED_WAITING_FOR_TURN
```

轮到该任务执行时：

```text
queue_position: 0
state: PLANNING_AND_EXECUTING
```

调度过程：

```text
任务 1 正在执行
任务 2 等待
任务 3 等待
    ↓
任务 1 完成
    ↓
任务 2 自动开始
    ↓
任务 2 完成
    ↓
任务 3 自动开始
```

---

## 11. Action 取消

ROS 2 Humble 的命令行工具未直接提供 `ros2 action cancel` 子命令，因此本项目通过 Action 隐藏取消服务实现任务取消。

取消当前活动任务：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/ros2_ws/install/setup.bash

ros2 service call \
  /openarm/queued_execute_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
```

取消成功后的客户端结果示例：

```text
state: CANCELING_MOVEIT_TRAJECTORY

Result:
  success: false
  error_code: -2
  message: 任务已由调用者取消

Goal finished with status: CANCELED
```

---

## 12. TF 闭环到达判定

项目通过 TF 获取右臂末端实际位姿：

```text
world → openarm_right_ee_base_link
```

查询末端实际位姿：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/ros2_ws/install/setup.bash

ros2 run tf2_ros tf2_echo \
  world \
  openarm_right_ee_base_link
```

输出示例：

```text
Translation: [0.028, -0.158, 0.265]

Rotation: in Quaternion (xyzw)
[-0.042, -0.025, -0.027, 0.998]

Rotation: in RPY (degree)
[-4.729, -3.015, -2.930]
```

| 数据 | 含义 | 单位 |
|---|---|---|
| `Translation x/y/z` | 末端位置 | 米 |
| `Quaternion x/y/z/w` | 末端朝向四元数 | 无单位 |
| `RPY` | 末端朝向的欧拉角表示 | 度或弧度 |

自动到达判定节点：

```text
pose_goal_reach_monitor.py
```

当前到达阈值：

| 指标 | 阈值 |
|---|---|
| 位置误差 | `≤ 0.005 m`，即 5 mm |
| 姿态误差 | `≤ 0.26 rad`，约 14.9° |

一次测试结果：

```text
目标位置：x=0.030 m，y=-0.154 m，z=0.262 m
位置误差：4.98 mm
姿态误差：6.38°
判定结果：已到达目标
```

---

## 13. Topic 位姿控制链路

项目还实现了基于 ROS 2 Topic 的控制接口：

```text
/openarm/target_pose
```

消息类型：

```text
geometry_msgs/msg/PoseStamped
```

启动“话题目标 → MoveIt 执行 → TF 验收”：

```bash
cd ~/robot_project/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch openarm_learning topic_pose_control.launch.py
```

发布一条末端 6D 位姿目标：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/ros2_ws/install/setup.bash

ros2 topic pub --once /openarm/target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.030, y: -0.154, z: 0.262}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

---

## 14. Planning Scene 与碰撞场景

项目通过 `planning_scene_obstacle.py` 调用 MoveIt 的：

```text
/apply_planning_scene
```

向规划场景添加或删除名为 `demo_box` 的虚拟碰撞盒。

添加障碍物：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/ros2_ws/install/setup.bash

ros2 run openarm_learning planning_scene_obstacle --ros-args \
  -p x:=0.060 \
  -p y:=-0.154 \
  -p z:=0.262 \
  -p size_x:=0.040 \
  -p size_y:=0.040 \
  -p size_z:=0.040
```

删除障碍物：

```bash
ros2 run openarm_learning planning_scene_obstacle --ros-args \
  -p remove:=true
```

查询规划场景中是否存在 `demo_box`：

```bash
ros2 service call /get_planning_scene \
  moveit_msgs/srv/GetPlanningScene \
  "{components: {components: 1023}}" \
  | grep -A 20 -B 2 "demo_box"
```

当前已验证：

- MoveIt 能接收并保存虚拟障碍物；
- RViz 的 `Scene Objects` 中可以显示 `demo_box`；
- 当末端目标或可行轨迹与障碍物冲突时，MoveIt 会拒绝规划；
- 自定义 Action 会返回失败状态，不会向控制器发送潜在碰撞轨迹；
- 当障碍物不影响当前规划时，MoveIt 可继续生成并执行轨迹。

> 注意：当前项目已验证“规划场景更新”和“碰撞导致的规划失败”。  
> 严格证明“机械臂绕过某个障碍物并到达同一目标”，需要基于完整机械臂各连杆的扫掠区域设计障碍物位置，并对比有无障碍物时的完整关节轨迹；该验证作为后续增强方向，当前不将其表述为已完成能力。

---

## 15. GitHub Actions 持续集成

仓库包含 GitHub Actions 工作流：

```text
.github/workflows/ros2-ci.yml
```

持续集成验证：

- ROS 2 Humble 环境是否可正常启动；
- `openarm_interfaces` 是否可以成功构建；
- `ExecutePose.action` 接口是否能够被识别；
- Python 节点是否通过基础语法检查；
- 仓库结构是否符合当前 ROS 2 包组织方式。

每次推送到 GitHub 后，可在仓库顶部的 **Actions** 页面查看构建状态。

---

## 16. 项目成果总结

本项目完成了从高层末端目标到低层机械臂执行、再到闭环验收的完整软件链路：

```text
6D 位姿目标
    ↓
ROS 2 Topic 或自定义 Action
    ↓
FIFO 任务队列与取消管理
    ↓
MoveIt 逆运动学、碰撞检测、轨迹规划
    ↓
ros2_control 控制器执行
    ↓
OpenArm 假硬件与 RViz 可视化
    ↓
TF 获取真实末端位姿
    ↓
位置与姿态误差自动验收
```

目前已具备：

- ROS 2 软件包与工作空间管理；
- URDF/Xacro 机器人模型使用；
- MoveIt 2 逆运动学与轨迹规划；
- ros2_control 控制器链路；
- TF 坐标变换与位姿监控；
- 四元数、RPY 与 6D 位姿表达；
- ROS 2 Topic、Service、Action 通信；
- Action 实时反馈、FIFO 队列和取消；
- Planning Scene 碰撞物体管理；
- Git/GitHub 与 GitHub Actions 基础持续集成。

---

## 17. 项目局限

- 当前使用 `mock_components/GenericSystem` 假硬件，不控制真实机械臂。
- 尚未接入 CAN 总线、电机驱动器、真实传感器和急停安全链路。
- 当前碰撞场景已能够阻止冲突规划，但严格的“绕障到达同一目标”对比实验尚未完成。
- 当前任务队列采用 FIFO 策略，尚未实现优先级、超时和重试策略。
- 当前目标输入来自命令行、Topic 或 Action，尚未集成视觉语言模型等高层决策模块。

---

## 18. 后续方向

1. 完成严格的绕障轨迹对比实验，验证障碍物影响下的可行替代轨迹。
2. 为队列增加任务优先级、超时、重试和失败恢复策略。
3. 增加更多单元测试和集成测试，完善持续集成覆盖范围。
4. 支持动态障碍物更新与规划场景实时刷新。
5. 将视觉语言模型输出映射为安全的 OpenArm 位姿任务，实现高层决策与底层控制集成。
6. 在完成安全验证后，接入真实 OpenArm 硬件与 CAN 通信。

---

## 19. 简历描述参考

> 基于 ROS 2 Humble、MoveIt 2 和 ros2_control 搭建 OpenArm 双臂假硬件仿真控制系统；实现右臂末端 6D 位姿控制、逆运动学求解、轨迹规划、控制器执行，以及基于 TF 的位置/姿态闭环误差验收。在测试中达到 4.98 mm 位置误差与 6.38° 姿态误差。设计自定义 ROS 2 Action 接口，支持最多 5 个任务 FIFO 排队、实时反馈与执行中取消；集成 MoveIt Planning Scene，实现虚拟碰撞物体管理与冲突规划拒绝，并通过 GitHub Actions 完成基础持续集成验证。