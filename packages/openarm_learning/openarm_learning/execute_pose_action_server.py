# 导入 Python 的线程锁，用于保证同一时间只执行一个 Action 任务。
from threading import Lock

# 导入 Python 的时间模块，用于在等待异步结果时短暂休眠。
import time

# 导入 ROS 2 Python 客户端库。
import rclpy

# 导入 ROS 2 Action 客户端、服务器及其接收/取消响应枚举。
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse

# 导入可重入回调组，让 Action 与 MoveIt 回调可以并行处理。
from rclpy.callback_groups import ReentrantCallbackGroup

# 导入多线程执行器，避免等待 MoveIt 结果时阻塞取消请求。
from rclpy.executors import MultiThreadedExecutor

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入几何消息中的 Pose，用于描述球形位置约束的姿态字段。
from geometry_msgs.msg import Pose

# 导入 MoveIt 的规划 Action。
from moveit_msgs.action import MoveGroup

# 导入 MoveIt 的约束与错误码消息。
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)

# 导入球体几何类型。
from shape_msgs.msg import SolidPrimitive

# 导入我们刚刚定义的自定义 Action。
from openarm_interfaces.action import ExecutePose


# 定义要由 MoveIt 控制的规划组名称。
PLANNING_GROUP = "right_arm"

# 定义右臂末端执行器对应的 TF/URDF 链接名称。
END_EFFECTOR_LINK = "openarm_right_ee_base_link"

# 定义位置目标允许的球形半径，单位为米。
POSITION_TOLERANCE = 0.005

# 定义末端朝向绕 X 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_X = 0.15

# 定义末端朝向绕 Y 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_Y = 0.15

# 定义末端朝向绕 Z 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_Z = 0.15


# 创建“末端位姿任务”的 Action 服务器节点。
class ExecutePoseActionServer(Node):

    # 初始化节点、MoveIt 客户端与自定义 Action 服务器。
    def __init__(self):
        # 初始化节点名称。
        super().__init__("execute_pose_action_server")

        # 创建可重入回调组，使等待 MoveIt 时仍能处理取消请求。
        self.callback_group = ReentrantCallbackGroup()

        # 创建互斥锁，保护 busy 状态。
        self.busy_lock = Lock()

        # 记录当前是否已有任务正在执行。
        self.busy = False

        # 创建调用 MoveIt /move_action 的 Action 客户端。
        self.moveit_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
            callback_group=self.callback_group,
        )

        # 创建对外提供的自定义 Action 服务器。
        self.action_server = ActionServer(
            self,
            ExecutePose,
            "/openarm/execute_pose",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        # 打印启动提示。
        self.get_logger().info(
            "Action 服务器已启动：/openarm/execute_pose"
        )

    # 收到新任务时决定接受还是拒绝。
    def goal_callback(self, goal_request):
        # 读取目标四元数各分量。
        orientation = goal_request.target_pose.pose.orientation

        # 计算四元数长度平方，避免接受全零的非法朝向。
        quaternion_norm_squared = (
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )

        # 如果四元数接近全零，则拒绝该目标。
        if quaternion_norm_squared < 1e-8:
            self.get_logger().error("拒绝目标：四元数不能为全零")
            return GoalResponse.REJECT

        # 加锁检查当前是否已有动作在执行。
        with self.busy_lock:
            # 如果繁忙，则拒绝新目标，避免多个 MoveIt 请求争抢右臂。
            if self.busy:
                self.get_logger().warn("拒绝目标：右臂当前正在执行另一个 Action")
                return GoalResponse.REJECT

            # 标记右臂开始忙碌。
            self.busy = True

        # 接受该任务。
        self.get_logger().info("已接受新的 6D 位姿 Action 目标")
        return GoalResponse.ACCEPT

    # 收到取消请求时决定是否允许取消。
    def cancel_callback(self, goal_handle):
        # 记录取消请求。
        self.get_logger().warn("收到取消请求，将停止当前 MoveIt 任务")

        # 允许 ROS 2 客户端取消当前任务。
        return CancelResponse.ACCEPT

    # 将自定义 Action 的目标转换成 MoveIt 请求并执行。
    def execute_callback(self, goal_handle):
        # 创建最终返回给调用者的 Result 对象。
        result = ExecutePose.Result()

        # 读取调用者给出的 PoseStamped 目标。
        target_pose_stamped = goal_handle.request.target_pose

        # 如果调用者未填写参考坐标系，默认采用 world。
        reference_frame = target_pose_stamped.header.frame_id or "world"

        # 读取完整目标位姿。
        target_pose = target_pose_stamped.pose

        # 创建反馈消息对象。
        feedback = ExecutePose.Feedback()

        # 当前版本一次只执行一条任务，因此队列位置固定为 0。
        feedback.queue_position = 0

        # 告知调用者正在构造 MoveIt 请求。
        feedback.state = "BUILDING_MOVEIT_GOAL"

        # 发布第一条反馈。
        goal_handle.publish_feedback(feedback)

        # 创建 MoveIt 的完整目标对象。
        moveit_goal = MoveGroup.Goal()

        # 指定右臂规划组。
        moveit_goal.request.group_name = PLANNING_GROUP

        # 使用机器人当前状态作为轨迹起点。
        moveit_goal.request.start_state.is_diff = True

        # 设置规划最大尝试次数。
        moveit_goal.request.num_planning_attempts = 5

        # 设置单次规划允许的最长时间，单位为秒。
        moveit_goal.request.allowed_planning_time = 5.0

        # 设置轨迹速度缩放比例。
        moveit_goal.request.max_velocity_scaling_factor = 0.10

        # 设置轨迹加速度缩放比例。
        moveit_goal.request.max_acceleration_scaling_factor = 0.10

        # 创建一个约束集合。
        constraints = Constraints()

        # 创建末端位置约束。
        position_constraint = PositionConstraint()

        # 指定位置约束的参考坐标系。
        position_constraint.header.frame_id = reference_frame

        # 指定被约束的末端链接。
        position_constraint.link_name = END_EFFECTOR_LINK

        # 设置位置约束的权重。
        position_constraint.weight = 1.0

        # 创建表示允许区域的球体。
        sphere = SolidPrimitive()

        # 指定几何体类型为球。
        sphere.type = SolidPrimitive.SPHERE

        # 指定球半径，即位置容差。
        sphere.dimensions = [POSITION_TOLERANCE]

        # 将球体加入位置约束区域。
        position_constraint.constraint_region.primitives = [sphere]

        # 将球心放在用户给定的目标位置。
        position_constraint.constraint_region.primitive_poses = [target_pose]

        # 创建末端方向约束。
        orientation_constraint = OrientationConstraint()

        # 指定方向约束的参考坐标系。
        orientation_constraint.header.frame_id = reference_frame

        # 指定被约束的末端链接。
        orientation_constraint.link_name = END_EFFECTOR_LINK

        # 指定期望四元数朝向。
        orientation_constraint.orientation = target_pose.orientation

        # 设置绕 X 轴允许的姿态误差。
        orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_X

        # 设置绕 Y 轴允许的姿态误差。
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_Y

        # 设置绕 Z 轴允许的姿态误差。
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_Z

        # 定义“任务被调用者取消”的自定义结果码。
        CANCELED_BY_CLIENT = -2

        # 设置方向约束权重。
        orientation_constraint.weight = 1.0

        # 将位置约束加入约束集合。
        constraints.position_constraints.append(position_constraint)

        # 将方向约束加入约束集合。
        constraints.orientation_constraints.append(orientation_constraint)

        # 把完整 6D 约束交给 MoveIt。
        moveit_goal.request.goal_constraints = [constraints]

        # 指定不仅规划，还要执行轨迹。
        moveit_goal.planning_options.plan_only = False

        # 允许 MoveIt 在需要时重新规划。
        moveit_goal.planning_options.replan = True

        # 指定最多重新规划两次。
        moveit_goal.planning_options.replan_attempts = 2

        # 等待 MoveIt Action 服务出现。
        if not self.moveit_client.wait_for_server(timeout_sec=5.0):
            # 标记任务失败。
            goal_handle.abort()

            # 填写失败结果。
            result.success = False
            result.error_code = MoveItErrorCodes.FAILURE
            result.message = "MoveIt /move_action 服务不可用"

            # 返回结果给调用者。
            return result

        # 更新反馈状态。
        feedback.state = "WAITING_MOVEIT_ACCEPTANCE"

        # 发布反馈。
        goal_handle.publish_feedback(feedback)

        # 异步发送 MoveIt 目标。
        moveit_goal_future = self.moveit_client.send_goal_async(moveit_goal)

        # 等待 MoveIt 接受或拒绝目标。
        while rclpy.ok() and not moveit_goal_future.done():
            # 短暂休眠，让其他 ROS 回调继续运行。
            time.sleep(0.05)

        # 读取 MoveIt 返回的目标句柄。
        moveit_goal_handle = moveit_goal_future.result()

        # 如果 MoveIt 拒绝目标，则结束任务。
        if not moveit_goal_handle.accepted:
            # 标记外层 Action 失败。
            goal_handle.abort()

            # 填写失败结果。
            result.success = False
            result.error_code = MoveItErrorCodes.FAILURE
            result.message = "MoveIt 拒绝了该目标"

            # 清除忙碌状态。
            self.finish_current_task()

            # 返回结果。
            return result

        # 更新反馈状态为规划和执行中。
        feedback.state = "PLANNING_AND_EXECUTING"

        # 发布反馈。
        goal_handle.publish_feedback(feedback)

        # 请求 MoveIt 返回最终执行结果。
        moveit_result_future = moveit_goal_handle.get_result_async()

        # 记录是否已经向 MoveIt 转发过取消请求。
        cancel_sent_to_moveit = False

        # 持续等待 MoveIt 最终结果。
        while rclpy.ok() and not moveit_result_future.done():
            # 如果调用者提出取消，且尚未转发给 MoveIt。
            if goal_handle.is_cancel_requested and not cancel_sent_to_moveit:
                # 更新反馈状态。
                feedback.state = "CANCELING_MOVEIT_TRAJECTORY"

                # 发布反馈。
                goal_handle.publish_feedback(feedback)

                # 向 MoveIt 转发取消请求。
                moveit_goal_handle.cancel_goal_async()

                # 标记已发送取消请求。
                cancel_sent_to_moveit = True

            # 短暂休眠，让取消等回调继续运行。
            time.sleep(0.05)

        # 读取 MoveIt 的最终封装结果。
        moveit_wrapped_result = moveit_result_future.result()

        # 读取 MoveIt 错误码数值。
        moveit_error_code = moveit_wrapped_result.result.error_code.val

        # 如果调用者请求过取消，则返回“已取消”。
        if goal_handle.is_cancel_requested:
            # 标记该 Action 已取消。
            goal_handle.canceled()

            # 填写取消结果。
            result.success = False

            # 使用本项目定义的取消码，而不是沿用底层 MoveIt 的结果码。
            result.error_code = CANCELED_BY_CLIENT

            # 说明任务由外部调用者主动取消。
            result.message = "任务已由调用者取消"   

        # 如果 MoveIt 成功规划并执行。
        elif moveit_error_code == MoveItErrorCodes.SUCCESS:
            # 标记外层 Action 成功。
            goal_handle.succeed()

            # 填写成功结果。
            result.success = True
            result.error_code = moveit_error_code
            result.message = "MoveIt 已成功规划并执行 6D 位姿任务"

        # 其余情况都视为任务失败。
        else:
            # 标记外层 Action 失败。
            goal_handle.abort()

            # 填写失败结果。
            result.success = False
            result.error_code = moveit_error_code
            result.message = "MoveIt 未能完成该 6D 位姿任务"

        # 清除忙碌状态。
        self.finish_current_task()

        # 返回最终结果给调用者。
        return result

    # 统一释放当前任务的忙碌标记。
    def finish_current_task(self):
        # 加锁修改 busy 状态。
        with self.busy_lock:
            # 标记右臂目前空闲。
            self.busy = False


# 定义 ROS 2 节点入口函数。
def main(args=None):
    # 初始化 ROS 2。
    rclpy.init(args=args)

    # 创建 Action 服务器节点。
    node = ExecutePoseActionServer()

    # 创建四线程执行器，保证执行、反馈和取消可并行处理。
    executor = MultiThreadedExecutor(num_threads=4)

    # 将节点加入执行器。
    executor.add_node(node)

    try:
        # 持续运行节点。
        executor.spin()
    finally:
        # 销毁 Action 服务器节点。
        node.destroy_node()

        # 关闭 ROS 2。
        # 仅在 ROS 2 上下文尚未关闭时才关闭，避免 Ctrl+C 后重复关闭报错。
        if rclpy.ok():
            # 正常关闭 ROS 2。
            rclpy.shutdown()


# 仅当该文件被直接运行时才进入 main。
if __name__ == "__main__":
    # 启动节点。
    main()