# 指定构建该接口包需要的最低 CMake 版本。
cmake_minimum_required(VERSION 3.8)

# 声明 ROS 2 软件包名称。
project(openarm_interfaces)

# 查找 ament 的基础构建工具。
find_package(ament_cmake REQUIRED)

# 查找 ROS 2 接口代码生成工具。
find_package(rosidl_default_generators REQUIRED)

# 查找 Pose、PoseStamped 等几何消息所属的软件包。
find_package(geometry_msgs REQUIRED)

# 查找 Header 消息所属的软件包。
find_package(std_msgs REQUIRED)

# 查找 Image 消息所属的软件包。
find_package(sensor_msgs REQUIRED)

# 根据 Action 和 Message 文件自动生成 Python、C++ 等语言可用的接口代码。
rosidl_generate_interfaces(${PROJECT_NAME}
  # 原有的末端位姿执行 Action。
  "action/ExecutePose.action"

  # 新增的 OpenArm 结构化观测消息。
  "msg/OpenArmObservation.msg"

  "msg/OpenArmExecutionEvent.msg"

  # 新增的真实 LingBot 实时影子推理结果消息。
  "msg/LingBotShadowResult.msg"

  # 新增的 LingBot 影子动作风险门控决策消息。
  "msg/LingBotShadowSafetyDecision.msg"

  # 声明上述接口文件中使用到的消息依赖。
  DEPENDENCIES
    geometry_msgs
    std_msgs
    sensor_msgs
)

# 导出运行时所需的 ROS 2 接口支持。
ament_export_dependencies(rosidl_default_runtime)

# 结束 ament 软件包配置。
ament_package()