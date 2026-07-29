# 导入双端队列，用于实现先进先出 FIFO 任务队列。
from collections import deque

# 导入数据类工具，用于保存每条队列任务的信息。
from dataclasses import dataclass

# 导入线程事件，用于唤醒等待执行的 Action 任务。
from threading import Event

# 导入线程锁，用于安全地访问共享队列。
from threading import Lock

# 导入时间模块，用于异步等待时短暂休眠。
import time

# 导入 ROS 2 Python 客户端库。
import rclpy

# 导入 ROS 2 Action 客户端、服务器及其响应枚举。
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse

# 导入可重入回调组，让多个任务、MoveIt 回调与取消请求可以并行处理。
from rclpy.callback_groups import ReentrantCallbackGroup

# 导入多线程执行器，避免排队任务阻塞 ROS 2 通信。
from rclpy.executors import MultiThreadedExecutor

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入 MoveIt 的规划 Action。
from moveit_msgs.action import MoveGroup

# 导入 MoveIt 的约束与错误码消息。
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)

# 导入球体几何消息，用于表示位置容差区域。
from shape_msgs.msg import SolidPrimitive

# 导入自定义的末端 6D 位姿 Action。
from openarm_interfaces.action import ExecutePose


# 定义由 MoveIt 控制的右臂规划组。
PLANNING_GROUP = "right_arm"

# 定义右臂末端执行器链接名称。
END_EFFECTOR_LINK = "openarm_right_ee_base_link"

# 定义位置目标的球形容差，单位为米。
POSITION_TOLERANCE = 0.005

# 定义末端朝向绕 X 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_X = 0.15

# 定义末端朝向绕 Y 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_Y = 0.15

# 定义末端朝向绕 Z 轴允许的误差，单位为弧度。
ORIENTATION_TOLERANCE_Z = 0.15

# 定义用户取消任务时返回的项目级错误码。
CANCELED_BY_CLIENT = -2

# 定义队列最多容纳的任务总数，包含正在执行的任务。
MAX_TOTAL_TASKS = 5


# 保存一条已被 Action 服务器接受的任务。
@dataclass
class QueuedTask:
    # 保存 ROS 2 Action 的目标句柄。
    goal_handle: object

    # 当前任务轮到执行时，由服务器设置该事件。
    start_event: Event


# 创建支持 FIFO 队列的 6D 位姿 Action 服务器。
class QueuedExecutePoseActionServer(Node):

    # 初始化节点、队列、MoveIt 客户端和 Action 服务器。
    def __init__(self):
        # 初始化 ROS 2 节点名称。
        super().__init__("queued_execute_pose_action_server")

        # 创建可重入回调组，允许并行接收目标、处理取消和等待 MoveIt。
        self.callback_group = ReentrantCallbackGroup()

        # 创建线程锁，保护队列与当前任务状态。
        self.queue_lock = Lock()

        # 创建等待队列，队首是下一条应执行的任务。
        self.waiting_tasks = deque()

        # 保存当前正在执行的任务；空闲时为 None。
        self.current_task = None

        # 记录已接受但尚未结束的任务总数，用于限制队列容量。
        self.reserved_task_count = 0

        # 创建调用 MoveIt /move_action 的 Action 客户端。
        self.moveit_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
            callback_group=self.callback_group,
        )

        # 创建对外暴露的队列式 Action 服务器。
        self.action_server = ActionServer(
            self,
            ExecutePose,
            "/openarm/queued_execute_pose",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        # 输出启动说明。
        self.get_logger().info(
            "队列式 Action 服务器已启动：/openarm/queued_execute_pose"
        )

    # 验证并决定是否接受新目标。
    def goal_callback(self, goal_request):
        # 读取目标四元数。
        orientation = goal_request.target_pose.pose.orientation

        # 计算四元数长度平方，防止接受全零四元数。
        quaternion_norm_squared = (
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )

        # 全零四元数不能表示有效朝向，因此直接拒绝。
        if quaternion_norm_squared < 1e-8:
            self.get_logger().error("拒绝目标：四元数不能为全零")
            return GoalResponse.REJECT

        # 加锁检查队列容量。
        with self.queue_lock:
            # 如果已达到最大任务数，则拒绝新任务。
            if self.reserved_task_count >= MAX_TOTAL_TASKS:
                self.get_logger().warn(
                    f"拒绝目标：任务队列已满，最大容量为 {MAX_TOTAL_TASKS}"
                )
                return GoalResponse.REJECT

            # 为即将接受的任务预留一个队列位置。
            self.reserved_task_count += 1

        # 接受该目标。
        self.get_logger().info("已接受新的队列式 6D 位姿任务")
        return GoalResponse.ACCEPT

    # 允许客户端请求取消已接受的目标。
    def cancel_callback(self, goal_handle):
        # 记录客户端的取消请求。
        self.get_logger().warn("收到 Action 取消请求")

        # 允许取消正在执行或仍在等待队列中的任务。
        return CancelResponse.ACCEPT

    # 注册任务，并在必要时将其设为当前执行任务。
    def register_task(self, task):
        # 加锁修改队列状态。
        with self.queue_lock:
            # 若当前没有任务执行，则当前任务直接开始。
            if self.current_task is None:
                # 将该任务设置为当前执行任务。
                self.current_task = task

                # 通知该任务可以立即开始执行。
                task.start_event.set()

                # 返回 0，代表“正在执行”。
                return 0

            # 如果已有任务执行，则将新任务加入队尾。
            self.waiting_tasks.append(task)

            # 返回等待位置；1 表示第一个等待任务。
            return len(self.waiting_tasks)

    # 向某条任务发送队列状态反馈。
    def publish_feedback(self, goal_handle, queue_position, state):
        # 创建 Action 反馈消息。
        feedback = ExecutePose.Feedback()

        # 设置任务在队列中的位置；0 代表正在执行。
        feedback.queue_position = queue_position

        # 设置人类可读的状态文本。
        feedback.state = state

        # 将反馈发送给该目标对应的客户端。
        goal_handle.publish_feedback(feedback)

    # 通知所有等待任务它们的最新队列位置。
    def publish_waiting_positions(self):
        # 加锁复制当前等待队列，避免遍历时被其他回调修改。
        with self.queue_lock:
            # 将等待任务复制为普通列表。
            tasks_snapshot = list(self.waiting_tasks)

        # 从 1 开始编号，因为 0 专门表示当前正在执行。
        for queue_position, task in enumerate(tasks_snapshot, start=1):
            # 向每个等待任务发送最新位置。
            self.publish_feedback(
                task.goal_handle,
                queue_position,
                "QUEUED_WAITING_FOR_TURN",
            )

    # 取消一条尚未开始执行的等待任务。
    def cancel_waiting_task(self, task):
        # 默认记录任务尚未被从等待队列移除。
        removed = False

        # 加锁访问等待队列。
        with self.queue_lock:
            # 若该任务仍然在等待队列中。
            if task in self.waiting_tasks:
                # 将该任务从等待队列移除。
                self.waiting_tasks.remove(task)

                # 释放该任务之前预留的容量。
                self.reserved_task_count -= 1

                # 记录移除成功。
                removed = True

        # 若移除了任务，则同步其余等待任务的位置。
        if removed:
            # 通知其余任务队列位置已变化。
            self.publish_waiting_positions()

        # 返回是否确实取消了等待任务。
        return removed

    # 当前任务结束后，自动启动下一条等待任务。
    def finish_current_task(self, task):
        # 加锁修改当前任务和队列状态。
        with self.queue_lock:
            # 仅当结束的是当前任务时才继续处理。
            if self.current_task is task:
                # 清空当前任务。
                self.current_task = None

                # 释放该任务占用的容量。
                self.reserved_task_count -= 1

                # 如果仍有等待任务。
                if self.waiting_tasks:
                    # 从队首取出下一条任务。
                    next_task = self.waiting_tasks.popleft()

                    # 将下一条任务设为当前执行任务。
                    self.current_task = next_task

                    # 唤醒下一条任务对应的执行回调。
                    next_task.start_event.set()

        # 通知仍在等待的任务，它们的位置可能已经前移。
        self.publish_waiting_positions()

    # 创建一个“调用者取消”结果。
    def create_canceled_result(self):
        # 创建 Action 最终结果对象。
        result = ExecutePose.Result()

        # 取消任务不属于成功。
        result.success = False

        # 使用项目定义的取消错误码。
        result.error_code = CANCELED_BY_CLIENT

        # 填写可读的取消原因。
        result.message = "任务已由调用者取消"

        # 返回取消结果。
        return result

    # 构造 MoveIt 的完整 6D 位姿规划请求。
    def build_moveit_goal(self, target_pose_stamped):
        # 读取参考坐标系；空字符串时默认使用 world。
        reference_frame = target_pose_stamped.header.frame_id or "world"

        # 读取调用者给出的完整 Pose。
        target_pose = target_pose_stamped.pose

        # 创建 MoveIt Action 目标对象。
        moveit_goal = MoveGroup.Goal()

        # 指定右臂规划组。
        moveit_goal.request.group_name = PLANNING_GROUP

        # 使用机器人执行时的当前状态作为轨迹起点。
        moveit_goal.request.start_state.is_diff = True

        # 设置最多尝试五次规划。
        moveit_goal.request.num_planning_attempts = 5

        # 设置单次规划最长五秒。
        moveit_goal.request.allowed_planning_time = 5.0

        # 限制速度比例，便于观察且更安全。
        moveit_goal.request.max_velocity_scaling_factor = 0.10

        # 限制加速度比例，便于观察且更安全。
        moveit_goal.request.max_acceleration_scaling_factor = 0.10

        # 创建末端约束集合。
        constraints = Constraints()

        # 创建位置约束。
        position_constraint = PositionConstraint()

        # 指定位置约束的参考坐标系。
        position_constraint.header.frame_id = reference_frame

        # 指定被约束的末端链接。
        position_constraint.link_name = END_EFFECTOR_LINK

        # 设置位置约束权重。
        position_constraint.weight = 1.0

        # 创建表示位置容差的球体。
        sphere = SolidPrimitive()

        # 指定几何体类型为球。
        sphere.type = SolidPrimitive.SPHERE

        # 设置球半径，即位置容差。
        sphere.dimensions = [POSITION_TOLERANCE]

        # 将球体加入位置约束区域。
        position_constraint.constraint_region.primitives = [sphere]

        # 将球心设为目标位置。
        position_constraint.constraint_region.primitive_poses = [target_pose]

        # 创建朝向约束。
        orientation_constraint = OrientationConstraint()

        # 指定朝向约束的参考坐标系。
        orientation_constraint.header.frame_id = reference_frame

        # 指定被约束的末端链接。
        orientation_constraint.link_name = END_EFFECTOR_LINK

        # 使用调用者给出的四元数作为期望朝向。
        orientation_constraint.orientation = target_pose.orientation

        # 设置绕 X 轴允许的姿态误差。
        orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_X

        # 设置绕 Y 轴允许的姿态误差。
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_Y

        # 设置绕 Z 轴允许的姿态误差。
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_Z

        # 设置朝向约束权重。
        orientation_constraint.weight = 1.0

        # 将位置约束加入约束集合。
        constraints.position_constraints.append(position_constraint)

        # 将朝向约束加入约束集合。
        constraints.orientation_constraints.append(orientation_constraint)

        # 将完整 6D 约束交给 MoveIt。
        moveit_goal.request.goal_constraints = [constraints]

        # 指定 MoveIt 在规划后直接执行。
        moveit_goal.planning_options.plan_only = False

        # 允许 MoveIt 必要时重新规划。
        moveit_goal.planning_options.replan = True

        # 最多允许重新规划两次。
        moveit_goal.planning_options.replan_attempts = 2

        # 返回构造完成的 MoveIt 目标。
        return moveit_goal

    # 执行当前已轮到的任务，并等待 MoveIt 最终结果。
    def execute_active_task(self, task):
        # 获取当前 Action 的目标句柄。
        goal_handle = task.goal_handle

        # 如果任务在刚开始时就已被取消。
        if goal_handle.is_cancel_requested:
            # 标记外层 Action 为已取消。
            goal_handle.canceled()

            # 返回取消结果。
            return self.create_canceled_result()

        # 发送“正在执行”反馈。
        self.publish_feedback(
            goal_handle,
            0,
            "PLANNING_AND_EXECUTING",
        )

        # 等待 MoveIt 的 /move_action 服务。
        if not self.moveit_client.wait_for_server(timeout_sec=5.0):
            # 标记 Action 失败。
            goal_handle.abort()

            # 创建失败结果。
            result = ExecutePose.Result()

            # 任务未成功。
            result.success = False

            # 使用 MoveIt 通用失败码。
            result.error_code = MoveItErrorCodes.FAILURE

            # 记录失败原因。
            result.message = "MoveIt /move_action 服务不可用"

            # 返回失败结果。
            return result

        # 根据调用者目标构造 MoveIt 请求。
        moveit_goal = self.build_moveit_goal(goal_handle.request.target_pose)

        # 异步发送 MoveIt 目标。
        moveit_goal_future = self.moveit_client.send_goal_async(moveit_goal)

        # 等待 MoveIt 接受或拒绝目标。
        while rclpy.ok() and not moveit_goal_future.done():
            # 短暂休眠，避免无意义占满 CPU。
            time.sleep(0.05)

        # 读取 MoveIt 目标句柄。
        moveit_goal_handle = moveit_goal_future.result()

        # 如果 MoveIt 拒绝了目标。
        if not moveit_goal_handle.accepted:
            # 若调用者已经请求取消，则优先返回取消状态。
            if goal_handle.is_cancel_requested:
                # 标记外层任务取消。
                goal_handle.canceled()

                # 返回取消结果。
                return self.create_canceled_result()

            # 否则标记任务失败。
            goal_handle.abort()

            # 创建失败结果。
            result = ExecutePose.Result()

            # 任务未成功。
            result.success = False

            # 使用通用失败码。
            result.error_code = MoveItErrorCodes.FAILURE

            # 说明 MoveIt 拒绝了目标。
            result.message = "MoveIt 拒绝了该目标"

            # 返回失败结果。
            return result

        # 获取 MoveIt 的异步最终结果。
        moveit_result_future = moveit_goal_handle.get_result_async()

        # 记录是否已向 MoveIt 转发取消请求。
        cancel_sent_to_moveit = False

        # 持续等待 MoveIt 执行结束。
        while rclpy.ok() and not moveit_result_future.done():
            # 如果调用者请求取消且尚未转发。
            if goal_handle.is_cancel_requested and not cancel_sent_to_moveit:
                # 发送取消中的反馈。
                self.publish_feedback(
                    goal_handle,
                    0,
                    "CANCELING_MOVEIT_TRAJECTORY",
                )

                # 向底层 MoveIt 转发取消请求。
                moveit_goal_handle.cancel_goal_async()

                # 标记取消请求已转发。
                cancel_sent_to_moveit = True

            # 短暂休眠，让 ROS 2 继续处理其他回调。
            time.sleep(0.05)

        # 读取 MoveIt 的封装结果。
        moveit_wrapped_result = moveit_result_future.result()

        # 提取底层 MoveIt 错误码。
        moveit_error_code = moveit_wrapped_result.result.error_code.val

        # 如果外层调用者请求取消。
        if goal_handle.is_cancel_requested:
            # 标记自定义 Action 已取消。
            goal_handle.canceled()

            # 返回项目级取消结果。
            return self.create_canceled_result()

        # 如果 MoveIt 成功规划并执行。
        if moveit_error_code == MoveItErrorCodes.SUCCESS:
            # 标记自定义 Action 成功。
            goal_handle.succeed()

            # 创建成功结果。
            result = ExecutePose.Result()

            # 标记任务成功。
            result.success = True

            # 返回底层 MoveIt 成功码 1。
            result.error_code = moveit_error_code

            # 填写成功信息。
            result.message = "MoveIt 已成功规划并执行队列任务"

            # 返回成功结果。
            return result

        # 其余情况都视为规划或执行失败。
        goal_handle.abort()

        # 创建失败结果。
        result = ExecutePose.Result()

        # 标记任务失败。
        result.success = False

        # 保留 MoveIt 返回的错误码。
        result.error_code = moveit_error_code

        # 填写失败信息。
        result.message = "MoveIt 未能完成该队列任务"

        # 返回失败结果。
        return result

    # 每条被接受的 Action 都会在独立回调中执行该函数。
    def execute_callback(self, goal_handle):
        # 创建该任务的启动事件。
        task = QueuedTask(
            goal_handle=goal_handle,
            start_event=Event(),
        )

        # 注册任务，并获取其初始队列位置。
        queue_position = self.register_task(task)

        # 若位置大于 0，说明任务需要排队等待。
        if queue_position > 0:
            # 发送排队反馈。
            self.publish_feedback(
                goal_handle,
                queue_position,
                "QUEUED_WAITING_FOR_TURN",
            )

            # 输出排队日志。
            self.get_logger().info(
                f"新任务已排队，当前等待位置为 {queue_position}"
            )

        # 持续等待任务轮到执行。
        while not task.start_event.wait(timeout=0.1):
            # 如果任务仍在等待阶段就被取消。
            if goal_handle.is_cancel_requested:
                # 尝试从等待队列移除该任务。
                removed = self.cancel_waiting_task(task)

                # 若移除成功，说明它确实还未开始执行。
                if removed:
                    # 标记该 Action 已取消。
                    goal_handle.canceled()

                    # 输出日志。
                    self.get_logger().info("等待中的任务已取消并移出队列")

                    # 返回取消结果。
                    return self.create_canceled_result()

        # 输出当前任务开始执行的日志。
        self.get_logger().info("当前任务开始交给 MoveIt 规划与执行")

        try:
            # 执行当前任务并等待最终结果。
            return self.execute_active_task(task)
        finally:
            # 无论成功、失败或取消，都自动调度下一条任务。
            self.finish_current_task(task)


# 定义 ROS 2 节点入口函数。
def main(args=None):
    # 初始化 ROS 2。
    rclpy.init(args=args)

    # 创建队列式 Action 服务器节点。
    node = QueuedExecutePoseActionServer()

    # 创建八线程执行器，支持多个等待任务与 MoveIt 回调并行。
    executor = MultiThreadedExecutor(num_threads=8)

    # 将节点加入执行器。
    executor.add_node(node)

    try:
        # 持续运行节点。
        executor.spin()
    finally:
        # 销毁节点。
        node.destroy_node()

        # 仅在上下文仍有效时关闭 ROS 2。
        if rclpy.ok():
            # 正常关闭 ROS 2。
            rclpy.shutdown()


# 仅在该文件被直接运行时启动节点。
if __name__ == "__main__":
    # 调用程序入口。
    main()