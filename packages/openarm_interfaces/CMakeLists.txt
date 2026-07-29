# 指定构建该接口包需要的最低 CMake 版本。
cmake_minimum_required(VERSION 3.8)

# 声明 ROS 2 软件包名称。
project(openarm_interfaces)

# 查找 ament 的基础构建工具。
find_package(ament_cmake REQUIRED)

# 查找 ROS 2 接口代码生成工具。
find_package(rosidl_default_generators REQUIRED)

# 查找 ExecutePose.action 中使用的 PoseStamped 消息所属包。
find_package(geometry_msgs REQUIRED)

# 根据 action 文件自动生成 Python、C++ 等语言可用的接口代码。
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecutePose.action"
  DEPENDENCIES geometry_msgs
)

# 导出运行时所需的 ROS 2 接口支持。
ament_export_dependencies(rosidl_default_runtime)

# 结束 ament 软件包配置。
ament_package()