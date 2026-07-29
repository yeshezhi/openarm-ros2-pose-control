# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入 Action 客户端，用于调用 MoveIt 的 /move_action。
from rclpy.action import ActionClient

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入带坐标系信息的末端位姿消息。
from geometry_msgs.msg import PoseStamped

# 导入 MoveIt 的规划 Action。
from moveit_msgs.action import MoveGroup

# 导入 MoveIt 的约束消息类型。
from moveit_msgs.msg import Constraints

# 导入末端位置约束消息类型。
from moveit_msgs.msg import PositionConstraint

# 导入末端朝向约束消息类型。
from moveit_msgs.msg import OrientationConstraint

# 导入 MoveIt 结果错误码消息类型。
from moveit_msgs.msg import MoveItErrorCodes

# 导入球体等几何体类型，用球体表示允许的位置误差范围。
from shape_msgs.msg import SolidPrimitive


# 定义要控制的 MoveIt 规划组，即右臂七个关节。
PLANNING_GROUP = 'right_arm'

# 定义右臂末端执行器对应的 TF 链接名称。
END_EFFECTOR_LINK = 'openarm_right_ee_base_link'

# 定义末端位置允许误差半径，单位为米。
POSITION_TOLERANCE = 0.005

# 定义末端朝向绕 X 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_X = 0.15

# 定义末端朝向绕 Y 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_Y = 0.15

# 定义末端朝向绕 Z 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_Z = 0.15


# 定义“话题目标位姿 → MoveIt 执行”节点。
class PoseGoalMoveItClient(Node):

    # 初始化节点。
    def __init__(self):
        # 创建 ROS 2 节点，并指定节点名称。
        super().__init__('pose_goal_moveit_client')

        # 创建 /move_action 的 Action 客户端。
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')

        # 订阅外部发布的目标位姿话题。
        self.subscription = self.create_subscription(
            # 指定消息类型为 PoseStamped。
            PoseStamped,
            # 指定订阅的目标话题名称。
            '/openarm/target_pose',
            # 每收到一条目标位姿就调用 pose_callback。
            self.pose_callback,
            # 设置最多缓存 10 条尚未处理的消息。
            10,
        )

        # 用于标记当前是否已有 MoveIt 目标正在执行。
        self.goal_in_progress = False

        # 输出节点启动提示。
        self.get_logger().info('等待 /openarm/target_pose，并将收到的目标发送给 MoveIt')

    # 收到一条目标位姿消息后自动执行。
    def pose_callback(self, message):
        # 如果上一条轨迹尚未结束，则拒绝新目标，避免两个规划请求互相干扰。
        if self.goal_in_progress:
            # 输出提示，说明当前机械臂仍在处理上一条命令。
            self.get_logger().warning('上一条 MoveIt 目标尚未完成，本次目标已忽略')

            # 直接结束本次回调函数。
            return

        # 等待 MoveIt 的 move_group 服务端就绪。
        self.move_action_client.wait_for_server()

        # 创建一个新的 MoveIt Action 目标。
        goal = MoveGroup.Goal()

        # 指定本次使用右臂规划组。
        goal.request.group_name = PLANNING_GROUP

        # 指定从机器人当前关节状态开始规划。
        goal.request.start_state.is_diff = True

        # 最多尝试五次寻找可行规划。
        goal.request.num_planning_attempts = 5

        # 单次规划最多允许五秒。
        goal.request.allowed_planning_time = 5.0

        # 将执行速度缩放为最大速度的 10%，便于观察。
        goal.request.max_velocity_scaling_factor = 0.10

        # 将执行加速度缩放为最大加速度的 10%，便于观察。
        goal.request.max_acceleration_scaling_factor = 0.10

        # 创建一组总约束，用于同时保存位置与朝向要求。
        constraints = Constraints()

        # 创建末端位置约束。
        position_constraint = PositionConstraint()

        # 使用发布者指定的参考坐标系，例如 world。
        position_constraint.header = message.header

        # 指定哪一个机械臂链接必须达到目标位置。
        position_constraint.link_name = END_EFFECTOR_LINK

        # 创建一个球体，球心就是目标位置。
        sphere = SolidPrimitive()

        # 指定该几何体为球体。
        sphere.type = SolidPrimitive.SPHERE

        # 指定球体半径，即允许的位置误差范围。
        sphere.dimensions = [POSITION_TOLERANCE]

        # 将球体添加到位置约束区域。
        position_constraint.constraint_region.primitives.append(sphere)

        # 将收到的目标位置作为球体中心。
        position_constraint.constraint_region.primitive_poses.append(message.pose)

        # 设定此位置约束的权重为 1。
        position_constraint.weight = 1.0

        # 创建末端朝向约束。
        orientation_constraint = OrientationConstraint()

        # 使用发布者指定的参考坐标系，例如 world。
        orientation_constraint.header = message.header

        # 指定哪一个链接必须满足该朝向。
        orientation_constraint.link_name = END_EFFECTOR_LINK

        # 直接使用消息中提供的四元数作为目标朝向。
        orientation_constraint.orientation = message.pose.orientation

        # 设置绕 X 轴允许的朝向误差。
        orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_X

        # 设置绕 Y 轴允许的朝向误差。
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_Y

        # 设置绕 Z 轴允许的朝向误差。
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_Z

        # 设定此朝向约束的权重为 1。
        orientation_constraint.weight = 1.0

        # 将位置约束添加到总约束中。
        constraints.position_constraints.append(position_constraint)

        # 将朝向约束添加到总约束中。
        constraints.orientation_constraints.append(orientation_constraint)

        # 将总约束作为本次 MoveIt 规划目标。
        goal.request.goal_constraints.append(constraints)

        # 指定不仅规划，还要自动执行规划结果。
        goal.planning_options.plan_only = False

        # 允许 MoveIt 在必要时重新规划。
        goal.planning_options.replan = True

        # 最多允许重新规划两次。
        goal.planning_options.replan_attempts = 2

        # 标记为“已有目标执行中”。
        self.goal_in_progress = True

        # 输出收到的三维目标位置。
        self.get_logger().info(
            f'收到并发送 6D 目标 | frame={message.header.frame_id} | '
            f'x={message.pose.position.x:.3f} m, '
            f'y={message.pose.position.y:.3f} m, '
            f'z={message.pose.position.z:.3f} m'
        )

        # 异步发送目标给 MoveIt，并注册“是否接受”的回调函数。
        send_goal_future = self.move_action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        # 当 MoveIt 回复是否接受时，调用 goal_response_callback。
        send_goal_future.add_done_callback(self.goal_response_callback)

    # 处理 MoveIt 是否接受目标的回复。
    def goal_response_callback(self, future):
        # 获取 MoveIt 返回的目标句柄。
        goal_handle = future.result()

        # 如果目标未被接受。
        if not goal_handle.accepted:
            # 输出错误信息。
            self.get_logger().error('MoveIt 拒绝了该目标')

            # 清除“执行中”标记。
            self.goal_in_progress = False

            # 结束函数。
            return

        # 输出提示，说明 MoveIt 已开始进行 IK、规划与执行。
        self.get_logger().info('MoveIt 已接受目标，正在进行 IK、规划与执行')

        # 异步等待最终结果。
        result_future = goal_handle.get_result_async()

        # 最终完成时调用 result_callback。
        result_future.add_done_callback(self.result_callback)

    # 接收 MoveIt 的过程状态反馈。
    def feedback_callback(self, feedback_message):
        # 读取 MoveIt 当前所处的状态文本。
        state = feedback_message.feedback.state

        # 输出当前状态，例如 PLANNING 或 MONITOR。
        self.get_logger().info(f'MoveIt 状态 | {state}')

    # 处理 MoveIt 的最终执行结果。
    def result_callback(self, future):
        # 读取 Action 返回结果中的 MoveIt 错误码。
        error_code = future.result().result.error_code.val

        # 建立常见错误码到中文说明的映射表。
        error_messages = {
            # 表示规划并执行成功。
            MoveItErrorCodes.SUCCESS: '规划并执行成功',
            # 表示没有找到满足目标约束的逆运动学解。
            MoveItErrorCodes.NO_IK_SOLUTION: '未找到可行的逆运动学解',
            # 表示轨迹规划失败。
            MoveItErrorCodes.PLANNING_FAILED: '轨迹规划失败',
            # 表示目标关节姿态会发生碰撞。
            MoveItErrorCodes.GOAL_IN_COLLISION: '目标状态发生碰撞',
            # 表示目标约束本身不合法。
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: '目标约束无效',
            # 表示规划超时。
            MoveItErrorCodes.TIMED_OUT: '规划超时',
            # 表示 MoveIt 返回了通用失败。
            MoveItErrorCodes.FAILURE: '通用失败：未找到可执行的 IK 或规划轨迹',
        }

        # 从映射表中取中文说明；未知错误码使用默认说明。
        message = error_messages.get(error_code, '未知 MoveIt 错误')

        # 如果结果码表示成功。
        if error_code == MoveItErrorCodes.SUCCESS:
            # 输出成功日志。
            self.get_logger().info(f'✅ MoveIt 执行完成：{message}')

        # 如果结果码不是成功。
        else:
            # 输出失败码与失败原因。
            self.get_logger().error(
                f'MoveIt 未完成目标 | code={error_code}，原因：{message}'
            )

        # 无论成功或失败，都解除“执行中”标记。
        self.goal_in_progress = False


# 定义程序入口函数。
def main():
    # 初始化 ROS 2 通信。
    rclpy.init()

    # 创建节点对象。
    node = PoseGoalMoveItClient()

    # 让节点持续运行并接收目标位姿。
    rclpy.spin(node)

    # 节点退出时释放资源。
    node.destroy_node()

    # 关闭 ROS 2 通信。
    rclpy.shutdown()


# 仅当该文件被直接运行时执行下面代码。
if __name__ == '__main__':
    # 启动程序。
    main()