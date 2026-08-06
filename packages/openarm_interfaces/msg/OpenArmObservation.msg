# 观测对应的 ROS 时间戳与参考坐标系。
std_msgs/Header header

# 提供给 VLA 的自然语言任务，例如“move the right end effector safely”。
string task_prompt

# OpenArm 右臂 joint1 到 joint7 的当前关节位置，单位为 rad。
float64[7] right_arm_joint_positions_rad

# world 坐标系下右臂末端的完整位姿：xyz（m）+ 四元数朝向（无单位）。
geometry_msgs/Pose end_effector_pose_world

# 右夹爪状态预留字段；当前没有接入真实夹爪时保持为 0.0。
float64 right_gripper_position

# 顶视相机是否已经接入并填充 camera_top。
bool has_camera_top

# 顶视 RGB 图像预留字段；当前未接相机时为空消息。
sensor_msgs/Image camera_top