# 本事件产生时的 ROS 时间戳。
std_msgs/Header header

# 本次动作决策所使用的观测时间戳与参考坐标系。
std_msgs/Header observation_header

# 本轮对应的自然语言任务。
string task_prompt

# LingBot 原始预测的第一步动作：[dx, dy, dz, droll, dpitch, dyaw]。
float64[6] requested_action_delta

# 安全适配器裁剪后的实际动作：[dx, dy, dz, droll, dpitch, dyaw]。
float64[6] safe_action_delta

# 安全动作转换得到的 world 坐标系末端目标位姿。
geometry_msgs/Pose target_pose_world

# MoveIt 是否完成规划和执行。
bool success

# MoveIt / Action 返回的错误码。
int32 error_code

# 便于日志和数据集检查的结果说明。
string message