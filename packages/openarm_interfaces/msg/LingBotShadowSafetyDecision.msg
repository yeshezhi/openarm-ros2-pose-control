# 本条安全分析结果产生时的 ROS 时间戳。
std_msgs/Header header

# 对应的 LingBot 影子结果时间戳。
std_msgs/Header shadow_result_header

# 对应的自然语言任务提示。
string task_prompt

# 当前安全模式；本项目固定为 shadow_analysis_only。
string mode

# 是否允许控制 OpenArm；当前必须始终为 false。
bool control_allowed

# 风险等级，可取 LOW、MEDIUM、HIGH 或 INVALID。
string risk_level

# RoboTwin 右臂六关节中最大的绝对目标变化量，单位为弧度。
float64 max_abs_right_arm_delta_rad

# 最大绝对目标变化量换算后的角度，单位为度。
float64 max_abs_right_arm_delta_deg

# 出现最大变化量的动作时间步下标。
uint32 worst_step_index

# 出现最大变化量的 RoboTwin 右臂关节下标，范围为 0 到 5。
uint32 worst_right_arm_joint_index

# 用于高风险判断的单关节变化阈值，单位为弧度。
float64 risk_threshold_rad

# 最终安全决策，例如 BLOCKED_CROSS_EMBODIMENT_HIGH_RISK。
string decision

# 所有阻断控制或风险提示原因。
string[] reasons
