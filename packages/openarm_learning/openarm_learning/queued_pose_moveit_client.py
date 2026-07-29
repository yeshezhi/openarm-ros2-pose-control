# 从 collections 导入双端队列，用于保存等待执行的目标。
from collections import deque

# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入 Action 客户端，用于调用 MoveIt 的 /move_action。
from rclpy.action import ActionClient

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入带坐标系信息的末端位姿消息。
from geometry_msgs.msg import PoseStamped

# 导入空消息，用作“取消当前任务”的命令。
from std_msgs.msg import Empty

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

# 导入球体等基本几何体消息。
from shape_msgs.msg import SolidPrimitive


# 定义要控制的 MoveIt 规划组。
PLANNING_GROUP = 'right_arm'

# 定义右臂末端参考链接。
END_EFFECTOR_LINK = 'openarm_right_ee_base_link'

# 定义位置容差半径，单位为米。
POSITION_TOLERANCE = 0.005

# 定义朝向绕 X 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_X = 0.15

# 定义朝向绕 Y 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_Y = 0.15

# 定义朝向绕 Z 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_Z = 0.15

# 定义最多缓存多少条尚未执行的目标。
MAX_QUEUE_SIZE = 5


# 定义支持目标队列和取消功能的 MoveIt 客户端。
class QueuedPoseMoveItClient(Node):

    # 初始化节点。
    def __init__(self):
        # 创建 ROS 2 节点。
        super().__init__('queued_pose_moveit_client')

        # 创建调用 MoveIt /move_action 的 Action 客户端。
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')

        # 订阅外部发布的末端 6D 位姿目标。
        self.target_subscription = self.create_subscription(
            # 指定消息类型。
            PoseStamped,
            # 指定目标位姿话题。
            '/openarm/target_pose',
            # 指定收到目标后的回调函数。
            self.pose_callback,
            # 设置接收队列长度。
            10,
        )

        # 订阅仅属于本节点的任务取消话题。
        self.cancel_subscription = self.create_subscription(
            # 指定空消息类型。
            Empty,
            # 指定取消命令话题。
            '/openarm/cancel_goal',
            # 指定收到取消命令后的回调函数。
            self.cancel_callback,
            # 设置接收队列长度。
            10,
        )

        # 创建保存等待目标的 FIFO 队列。
        self.goal_queue = deque()

        # 记录当前是否已有目标正在执行。
        self.goal_in_progress = False

        # 保存当前 MoveIt Action 目标句柄，用于只取消本节点自己的目标。
        self.current_goal_handle = None

        # 记录当前是否已收到取消请求。
        self.cancel_requested = False

        # 记录当前任务的序号。
        self.current_goal_number = 0

        # 输出节点启动提示。
        self.get_logger().info(
            f'正在监听 /openarm/target_pose，最多缓存 {MAX_QUEUE_SIZE} 个等待目标'
        )

        # 输出取消接口提示。
        self.get_logger().info('发布 Empty 消息到 /openarm/cancel_goal 可取消当前任务并清空队列')

    # 收到目标位姿消息时自动执行。
    def pose_callback(self, message):
        # 如果当前正在取消任务，则不再接收新目标。
        if self.cancel_requested:
            # 输出忽略提示。
            self.get_logger().warning('正在取消当前任务，本次新目标已忽略')

            # 结束本次回调。
            return

        # 如果当前没有目标正在执行。
        if not self.goal_in_progress:
            # 立即启动这条目标。
            self.start_goal(message)

            # 结束本次回调。
            return

        # 如果等待队列已满。
        if len(self.goal_queue) >= MAX_QUEUE_SIZE:
            # 输出拒绝提示，避免无限积压任务。
            self.get_logger().warning(
                f'目标队列已满，最多允许 {MAX_QUEUE_SIZE} 个等待目标，本次目标已拒绝'
            )

            # 结束本次回调。
            return

        # 将新目标追加到队列尾部。
        self.goal_queue.append(message)

        # 输出排队信息。
        self.get_logger().info(
            f'当前目标执行中，新目标已排队 | 等待数量={len(self.goal_queue)}'
        )

    # 收到取消命令时自动执行。
    def cancel_callback(self, message):
        # message 是 Empty 类型，仅表示取消命令到达，不携带额外数据。
        del message

        # 如果已经在处理取消命令。
        if self.cancel_requested:
            # 输出重复取消提示。
            self.get_logger().warning('取消请求正在处理中')

            # 结束本次回调。
            return

        # 记录取消前等待队列中的目标数量。
        pending_count = len(self.goal_queue)

        # 清空所有尚未执行的目标。
        self.goal_queue.clear()

        # 如果当前没有执行中的目标。
        if not self.goal_in_progress:
            # 输出取消结果。
            self.get_logger().info(
                f'当前没有执行中的目标，已清空 {pending_count} 个等待目标'
            )

            # 结束本次回调。
            return

        # 标记已经收到取消请求。
        self.cancel_requested = True

        # 输出取消命令与清空队列信息。
        self.get_logger().warning(
            f'收到取消命令，已清空 {pending_count} 个等待目标'
        )

        # 如果目标刚发送、尚未拿到 MoveIt 目标句柄。
        if self.current_goal_handle is None:
            # 输出等待提示。
            self.get_logger().info('正在等待 MoveIt 接受目标后立即发送取消请求')

            # 结束本次回调。
            return

        # 请求取消当前 MoveIt 目标。
        self.request_current_goal_cancel()

    # 向 MoveIt 发送取消当前目标的请求。
    def request_current_goal_cancel(self):
        # 异步请求取消当前目标。
        cancel_future = self.current_goal_handle.cancel_goal_async()

        # 注册取消请求响应回调。
        cancel_future.add_done_callback(self.cancel_response_callback)

    # 处理 MoveIt 对取消请求的响应。
    def cancel_response_callback(self, future):
        # 读取取消响应。
        response = future.result()

        # 如果 MoveIt 已接受至少一个目标的取消请求。
        if response.goals_canceling:
            # 输出取消已接受提示。
            self.get_logger().info('MoveIt 已接受取消请求，正在停止当前轨迹')

        # 如果 MoveIt 没有接受取消请求。
        else:
            # 输出提示，当前目标可能已自然结束。
            self.get_logger().warning('MoveIt 未接受取消请求，当前目标可能已结束')

    # 将一条目标位姿转换为 MoveIt 规划请求并发送。
    def start_goal(self, message):
        # 等待 MoveIt 服务端就绪。
        self.move_action_client.wait_for_server()

        # 标记当前已有目标正在执行。
        self.goal_in_progress = True

        # 清空上一条任务的目标句柄。
        self.current_goal_handle = None

        # 当前执行任务序号加一。
        self.current_goal_number += 1

        # 创建 MoveIt Action 目标。
        goal = MoveGroup.Goal()

        # 指定本次使用右臂规划组。
        goal.request.group_name = PLANNING_GROUP

        # 从机器人当前实际关节状态开始规划。
        goal.request.start_state.is_diff = True

        # 最多尝试五次寻找可行规划。
        goal.request.num_planning_attempts = 5

        # 每次规划最多允许五秒。
        goal.request.allowed_planning_time = 5.0

        # 使用最大速度的 10%，便于观察。
        goal.request.max_velocity_scaling_factor = 0.10

        # 使用最大加速度的 10%，便于观察。
        goal.request.max_acceleration_scaling_factor = 0.10

        # 创建总约束对象。
        constraints = Constraints()

        # 创建末端位置约束。
        position_constraint = PositionConstraint()

        # 使用消息中指定的参考坐标系。
        position_constraint.header = message.header

        # 指定必须到达目标位置的末端链接。
        position_constraint.link_name = END_EFFECTOR_LINK

        # 创建表示位置容差范围的球体。
        sphere = SolidPrimitive()

        # 指定几何体类型为球体。
        sphere.type = SolidPrimitive.SPHERE

        # 设置球体半径，即位置允许误差。
        sphere.dimensions = [POSITION_TOLERANCE]

        # 将球体加入位置约束区域。
        position_constraint.constraint_region.primitives.append(sphere)

        # 将目标位置作为球体中心。
        position_constraint.constraint_region.primitive_poses.append(message.pose)

        # 设置位置约束权重。
        position_constraint.weight = 1.0

        # 创建末端朝向约束。
        orientation_constraint = OrientationConstraint()

        # 使用消息中指定的参考坐标系。
        orientation_constraint.header = message.header

        # 指定必须满足朝向约束的末端链接。
        orientation_constraint.link_name = END_EFFECTOR_LINK

        # 使用消息中的四元数作为目标朝向。
        orientation_constraint.orientation = message.pose.orientation

        # 设置绕 X 轴允许的朝向误差。
        orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_X

        # 设置绕 Y 轴允许的朝向误差。
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_Y

        # 设置绕 Z 轴允许的朝向误差。
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_Z

        # 设置朝向约束权重。
        orientation_constraint.weight = 1.0

        # 将位置约束加入总约束。
        constraints.position_constraints.append(position_constraint)

        # 将朝向约束加入总约束。
        constraints.orientation_constraints.append(orientation_constraint)

        # 将总约束作为本次规划目标。
        goal.request.goal_constraints.append(constraints)

        # 指定规划完成后自动执行。
        goal.planning_options.plan_only = False

        # 允许 MoveIt 在必要时重新规划。
        goal.planning_options.replan = True

        # 最多允许重新规划两次。
        goal.planning_options.replan_attempts = 2

        # 输出当前任务序号、目标位置和队列数量。
        self.get_logger().info(
            f'开始执行目标 #{self.current_goal_number} | '
            f'x={message.pose.position.x:.3f} m, '
            f'y={message.pose.position.y:.3f} m, '
            f'z={message.pose.position.z:.3f} m | '
            f'等待数量={len(self.goal_queue)}'
        )

        # 异步发送目标给 MoveIt。
        send_goal_future = self.move_action_client.send_goal_async(
            # 发送构建好的 MoveIt 目标。
            goal,
            # 注册过程状态反馈回调。
            feedback_callback=self.feedback_callback,
        )

        # 当 MoveIt 回复是否接受目标时调用回调函数。
        send_goal_future.add_done_callback(self.goal_response_callback)

    # 处理 MoveIt 是否接受目标的回复。
    def goal_response_callback(self, future):
        # 获取 MoveIt 返回的目标句柄。
        goal_handle = future.result()

        # 如果 MoveIt 拒绝该目标。
        if not goal_handle.accepted:
            # 输出错误日志。
            self.get_logger().error('MoveIt 拒绝当前目标')

            # 处理当前任务结束。
            self.finish_current_goal(success=False)

            # 结束函数。
            return

        # 保存当前目标句柄，供取消功能使用。
        self.current_goal_handle = goal_handle

        # 输出接受提示。
        self.get_logger().info('MoveIt 已接受当前目标，正在进行 IK、规划与执行')

        # 异步等待 MoveIt 的最终结果。
        result_future = goal_handle.get_result_async()

        # 最终结果到达时调用 result_callback。
        result_future.add_done_callback(self.result_callback)

        # 如果目标接受前已经收到取消命令。
        if self.cancel_requested:
            # 现在拿到目标句柄后，立即请求取消。
            self.request_current_goal_cancel()

    # 接收 MoveIt 的规划过程状态。
    def feedback_callback(self, feedback_message):
        # 读取 MoveIt 当前状态。
        state = feedback_message.feedback.state

        # 输出当前状态，例如 PLANNING 或 IDLE。
        self.get_logger().info(f'MoveIt 状态 | {state}')

    # 处理 MoveIt 的最终执行结果。
    def result_callback(self, future):
        # 读取 MoveIt 返回的错误码。
        error_code = future.result().result.error_code.val

        # 如果当前流程属于用户主动取消。
        if self.cancel_requested:
            # 输出当前目标已结束的取消提示。
            self.get_logger().warning(
                f'当前目标取消流程结束 | MoveIt 返回 code={error_code}'
            )

            # 按取消逻辑结束当前任务。
            self.finish_current_goal(success=False, cancelled=True)

            # 结束函数。
            return

        # 如果错误码表示成功。
        if error_code == MoveItErrorCodes.SUCCESS:
            # 输出当前目标成功提示。
            self.get_logger().info(f'✅ 目标 #{self.current_goal_number} 执行成功')

            # 处理成功后的队列调度。
            self.finish_current_goal(success=True)

        # 如果错误码表示失败。
        else:
            # 输出当前目标失败信息。
            self.get_logger().error(
                f'目标 #{self.current_goal_number} 执行失败 | code={error_code}'
            )

            # 失败时停止后续队列，避免机械臂在未知状态继续执行。
            self.finish_current_goal(success=False)

    # 处理当前目标结束后的队列逻辑。
    def finish_current_goal(self, success, cancelled=False):
        # 保存取消请求状态，供后续逻辑判断。
        was_cancel_requested = self.cancel_requested

        # 标记当前不再有目标执行中。
        self.goal_in_progress = False

        # 清空当前目标句柄。
        self.current_goal_handle = None

        # 清除取消请求状态。
        self.cancel_requested = False

        # 如果本次是主动取消流程。
        if cancelled or was_cancel_requested:
            # 记录尚未执行目标数量。
            pending_count = len(self.goal_queue)

            # 再次确保等待队列为空。
            self.goal_queue.clear()

            # 输出取消完成提示。
            self.get_logger().warning(
                f'已结束当前任务，等待队列已清空 | 剩余等待目标={pending_count}'
            )

            # 结束函数。
            return

        # 如果当前目标失败。
        if not success:
            # 记录尚未执行目标数量。
            pending_count = len(self.goal_queue)

            # 清空等待队列，保证失败后安全停止。
            self.goal_queue.clear()

            # 输出安全停止提示。
            self.get_logger().warning(
                f'当前目标失败，已停止任务队列并清空 {pending_count} 个等待目标'
            )

            # 结束函数。
            return

        # 如果当前目标成功且队列为空。
        if not self.goal_queue:
            # 输出队列完成提示。
            self.get_logger().info('✅ 所有目标均已执行完毕')

            # 结束函数。
            return

        # 从队列头部取出下一条目标。
        next_goal = self.goal_queue.popleft()

        # 输出即将执行下一条目标的提示。
        self.get_logger().info(
            f'当前目标完成，自动执行下一条 | 剩余等待数量={len(self.goal_queue)}'
        )

        # 启动下一条目标。
        self.start_goal(next_goal)


# 定义程序入口函数。
def main():
    # 初始化 ROS 2 通信。
    rclpy.init()

    # 创建连续目标队列客户端节点。
    node = QueuedPoseMoveItClient()

    # 让节点持续运行并接收目标和取消命令。
    rclpy.spin(node)

    # 节点结束时释放资源。
    node.destroy_node()

    # 关闭 ROS 2 通信。
    rclpy.shutdown()


# 仅当该文件被直接运行时执行。
if __name__ == '__main__':
    # 启动程序入口。
    main()