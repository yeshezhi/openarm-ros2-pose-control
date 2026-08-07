# 本条实时影子推理结果产生时的 ROS 时间戳。
std_msgs/Header header

# 本次真实 LingBot 推理所使用的 OpenArm 观测时间戳。
std_msgs/Header observation_header

# 本轮自然语言任务提示。
string task_prompt

# 当前固定为 real_time_shadow_only，明确表示影子模式。
string mode

# 是否允许模型动作进入 OpenArm 控制链路；当前必须始终为 false。
bool control_enabled

# 本次真实模型实际使用的 14 维 RoboTwin 临时兼容状态。
float64[14] robotwin_compatible_state

# LingBot 返回的动作块时间步数，例如 3。
uint32 action_steps

# LingBot 每个时间步的动作维度，例如 RoboTwin 的 14。
uint32 action_dimension

# 完整动作块按行展平后的数据，长度等于 action_steps * action_dimension。
float64[] action_chunk

# 本次真实模型推理耗时，单位为毫秒。
float64 inference_ms

# 服务端上一条请求的完整处理耗时，单位为毫秒。
float64 previous_total_ms

# 当前 OpenArm 右臂实际关节数量。
uint32 openarm_joint_count

# 未被映射到 RoboTwin 的 OpenArm 关节下标；当前为 6。
int32 unmapped_openarm_joint_index

# 阻止模型动作直接控制 OpenArm 的安全原因。
string control_blocked_reason
