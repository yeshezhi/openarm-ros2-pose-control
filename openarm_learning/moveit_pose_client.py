# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入 Action 客户端类，用于调用 MoveIt 的规划 Action。
from rclpy.action import ActionClient

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入 ROS 2 时间类型，用于查询最新 TF。
from rclpy.time import Time

# 导入 Pose 消息，用于描述目标球中心位姿。
from geometry_msgs.msg import Pose

# 导入 MoveIt 的规划并执行 Action。
from moveit_msgs.action import MoveGroup

# 导入 MoveIt 的位置、朝向约束和错误码消息。
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)

# 导入球体几何消息，用于定义位置允许范围。
from shape_msgs.msg import SolidPrimitive

# 导入 TF 缓冲区、监听器和异常类型。
from tf2_ros import Buffer, TransformException, TransformListener


# 定义 MoveIt 的右臂规划组名称。
PLANNING_GROUP = "right_arm"

# 定义右臂末端参考 link 名称。
END_EFFECTOR_LINK = "openarm_right_ee_base_link"

# 定义规划目标所使用的世界坐标系。
REFERENCE_FRAME = "world"

# 定义相对当前位置沿世界 X 轴移动的距离，单位为米。
TARGET_OFFSET_X = 0.03

# 定义位置目标球的半径，0.010 米就是 1 厘米。
POSITION_TOLERANCE = 0.005

# 定义末端绕 X 轴允许的姿态误差，单位为弧度。
ORIENTATION_TOLERANCE_X = 0.15

# 定义末端绕 Y 轴允许的姿态误差，单位为弧度。
ORIENTATION_TOLERANCE_Y = 0.15

# 定义末端绕 Z 轴允许的姿态误差，单位为弧度。
ORIENTATION_TOLERANCE_Z = 0.15


# 定义一个节点，用于发送完整的末端 6D 位姿目标。
class MoveItPoseClient(Node):

    # 初始化节点、TF 工具和 MoveIt Action 客户端。
    def __init__(self):
        # 调用父类构造函数，并设置节点名称。
        super().__init__("moveit_pose_client")

        # 创建 TF 缓冲区，用来保存当前末端位姿。
        self.tf_buffer = Buffer()

        # 创建 TF 监听器，让缓冲区接收 /tf 数据。
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建 MoveIt Action 客户端，并连接 /move_action。
        self.action_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )

        # 记录是否已发送请求，避免定时器重复发送。
        self.goal_sent = False

        # 记录任务是否完成，供 main 函数判断何时退出。
        self.goal_finished = False

        # 记录上一条 MoveIt 状态，避免重复打印。
        self.last_feedback_state = ""

        # 每隔 0.5 秒尝试读取 TF 并发送一次目标。
        self.prepare_timer = self.create_timer(
            0.5,
            self.prepare_and_send_goal,
        )

    # 等待 TF 就绪后，读取当前 Pose 并发送规划请求。
    def prepare_and_send_goal(self):
        # 如果已经发送过目标，则不再重复发送。
        if self.goal_sent:
            # 结束当前函数。
            return

        # 等待 MoveIt 的 Action 服务端最多 5 秒。
        server_ready = self.action_client.wait_for_server(timeout_sec=5.0)

        # 如果 move_group 未启动，则输出错误并结束。
        if not server_ready:
            # 输出错误信息。
            self.get_logger().error("未找到 MoveIt 的 /move_action 服务端")

            # 标记任务已经结束。
            self.goal_finished = True

            # 取消准备定时器。
            self.prepare_timer.cancel()

            # 结束当前函数。
            return

        # 尝试读取 world 到右侧末端的最新 TF。
        try:
            # 查询当前末端完整位姿。
            transform = self.tf_buffer.lookup_transform(
                REFERENCE_FRAME,
                END_EFFECTOR_LINK,
                Time(),
            )

        # 如果 TF 尚未准备好，则等待下一次定时器再试。
        except TransformException as error:
            # 输出等待提示。
            self.get_logger().info(f"等待末端 TF: {error}")

            # 结束当前函数。
            return

        # 读取当前末端位置。
        current_position = transform.transform.translation

        # 读取当前末端朝向四元数。
        current_orientation = transform.transform.rotation

        # 调用函数，使用当前朝向创建一个小幅移动的 6D 目标。
        self.send_pose_goal(current_position, current_orientation)

        # 标记已经发送目标。
        self.goal_sent = True

        # 取消准备定时器，防止再次发送。
        self.prepare_timer.cancel()

    # 根据当前位置和当前朝向创建完整的 MoveIt 目标。
    def send_pose_goal(self, current_position, current_orientation):
        # 创建 MoveGroup 的目标对象。
        goal = MoveGroup.Goal()

        # 指定这次规划控制右臂。
        goal.request.group_name = PLANNING_GROUP

        # 让 MoveIt 从当前真实关节状态开始规划。
        goal.request.start_state.is_diff = True

        # 设置最多尝试 5 次规划。
        goal.request.num_planning_attempts = 5

        # 设置每次规划最多使用 5 秒。
        goal.request.allowed_planning_time = 5.0

        # 设置最大速度缩放为 10%。
        goal.request.max_velocity_scaling_factor = 0.10

        # 设置最大加速度缩放为 10%。
        goal.request.max_acceleration_scaling_factor = 0.10

        # 创建末端位置约束。
        position_constraint = PositionConstraint()

        # 指定位置目标使用 world 坐标系。
        position_constraint.header.frame_id = REFERENCE_FRAME

        # 指定被约束的是右侧末端 link。
        position_constraint.link_name = END_EFFECTOR_LINK

        # 创建一个表示允许位置范围的球体。
        target_sphere = SolidPrimitive()

        # 指定该几何体类型为球体。
        target_sphere.type = SolidPrimitive.SPHERE

        # 设置球体半径为 1 厘米。
        target_sphere.dimensions = [POSITION_TOLERANCE]

        # 创建球体中心的位姿。
        target_pose = Pose()

        # 将目标 X 坐标设为当前位置加 1 厘米。
        target_pose.position.x = current_position.x + TARGET_OFFSET_X

        # 保持目标 Y 坐标与当前位置相同。
        target_pose.position.y = current_position.y

        # 保持目标 Z 坐标与当前位置相同。
        target_pose.position.z = current_position.z

        # 设置球体位姿的单位四元数。
        target_pose.orientation.w = 1.0

        # 将目标球加入位置约束区域。
        position_constraint.constraint_region.primitives = [target_sphere]

        # 将目标球中心位姿加入位置约束区域。
        position_constraint.constraint_region.primitive_poses = [target_pose]

        # 设置位置约束权重为 1。
        position_constraint.weight = 1.0

        # 创建末端朝向约束。
        orientation_constraint = OrientationConstraint()

        # 指定朝向约束使用 world 坐标系。
        orientation_constraint.header.frame_id = REFERENCE_FRAME

        # 指定被约束的是右侧末端 link。
        orientation_constraint.link_name = END_EFFECTOR_LINK

        # 将当前四元数设为目标朝向的 qx。
        orientation_constraint.orientation.x = current_orientation.x

        # 将当前四元数设为目标朝向的 qy。
        orientation_constraint.orientation.y = current_orientation.y

        # 将当前四元数设为目标朝向的 qz。
        orientation_constraint.orientation.z = current_orientation.z

        # 将当前四元数设为目标朝向的 qw。
        orientation_constraint.orientation.w = current_orientation.w

        # 设置绕 X 轴允许的姿态误差。
        orientation_constraint.absolute_x_axis_tolerance = (
            ORIENTATION_TOLERANCE_X
        )

        # 设置绕 Y 轴允许的姿态误差。
        orientation_constraint.absolute_y_axis_tolerance = (
            ORIENTATION_TOLERANCE_Y
        )

        # 设置绕 Z 轴允许的姿态误差。
        orientation_constraint.absolute_z_axis_tolerance = (
            ORIENTATION_TOLERANCE_Z
        )

        # 设置朝向约束权重为 1。
        orientation_constraint.weight = 1.0

        # 创建一组目标约束。
        goal_constraints = Constraints()

        # 将位置约束加入目标约束组。
        goal_constraints.position_constraints = [position_constraint]

        # 将朝向约束加入目标约束组。
        goal_constraints.orientation_constraints = [orientation_constraint]

        # 将目标约束组交给 MoveIt。
        goal.request.goal_constraints = [goal_constraints]

        # 设置为 false，表示规划成功后直接执行。
        goal.planning_options.plan_only = False

        # 允许环境改变时重新规划。
        goal.planning_options.replan = True

        # 最多允许重新规划两次。
        goal.planning_options.replan_attempts = 2

        # 将规划场景设为增量更新。
        goal.planning_options.planning_scene_diff.is_diff = True

        # 将机器人状态设为增量读取当前状态。
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        # 输出目标位置和“保持当前朝向”的说明。
        self.get_logger().info(
            "发送完整 6D 位姿目标 | "
            f"x={target_pose.position.x:.3f} m, "
            f"y={target_pose.position.y:.3f} m, "
            f"z={target_pose.position.z:.3f} m，"
            "朝向保持为当前四元数"
        )

        # 异步发送目标，并注册反馈回调函数。
        send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        # 注册目标接受或拒绝的处理函数。
        send_goal_future.add_done_callback(self.goal_response_callback)

    # 处理 MoveIt 对目标的接受或拒绝。
    def goal_response_callback(self, future):
        # 取出 MoveIt 返回的目标句柄。
        goal_handle = future.result()

        # 如果 MoveIt 拒绝目标，则输出错误并结束。
        if not goal_handle.accepted:
            # 输出拒绝信息。
            self.get_logger().error("MoveIt 拒绝了该完整位姿目标")

            # 标记任务已经结束。
            self.goal_finished = True

            # 结束当前函数。
            return

        # 输出 MoveIt 已接受请求的信息。
        self.get_logger().info("MoveIt 已接受目标，正在执行 IK、规划与轨迹执行")

        # 异步等待最终规划和执行结果。
        result_future = goal_handle.get_result_async()

        # 注册最终结果处理函数。
        result_future.add_done_callback(self.result_callback)

    # 处理 MoveIt 规划和执行过程中的状态反馈。
    def feedback_callback(self, feedback_message):
        # 读取 MoveIt 当前状态字符串。
        state = feedback_message.feedback.state

        # 仅当状态变化时打印，避免重复刷屏。
        if state != self.last_feedback_state:
            # 输出当前状态。
            self.get_logger().info(f"MoveIt 状态 | {state}")

            # 保存当前状态。
            self.last_feedback_state = state

    # 处理 MoveIt 的最终结果。
        # 处理 MoveIt 的最终结果。
    def result_callback(self, future):
        # 取出完整 Action 结果。
        action_result = future.result()

        # 取出 MoveIt 的规划结果。
        result = action_result.result

        # 读取 MoveIt 返回的整数错误码。
        error_code = result.error_code.val

        # 建立常见错误码与中文解释的对应表。
        error_messages = {
            # 表示规划并执行成功。
            MoveItErrorCodes.SUCCESS: "规划并执行成功",

            # 表示 MoveIt 没有找到满足目标的逆运动学解。
            MoveItErrorCodes.NO_IK_SOLUTION: "未找到可行的逆运动学解",

            # 表示 MoveIt 无法生成一条有效轨迹。
            MoveItErrorCodes.PLANNING_FAILED: "轨迹规划失败",

            # 表示目标状态会导致机器人与环境或自身碰撞。
            MoveItErrorCodes.GOAL_IN_COLLISION: "目标状态发生碰撞",

            # 表示目标约束本身不合法。
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "目标约束无效",

            # 表示规划时间超过允许上限。
            MoveItErrorCodes.TIMED_OUT: "规划超时",
            
            # MoveIt 返回的通用失败码，通常表示规划器未能找到可执行解。
            MoveItErrorCodes.FAILURE: "通用失败：未找到可执行的 IK 或规划轨迹",
        }

        # 根据错误码获取中文解释；未知错误保留默认文字。
        error_message = error_messages.get(
            error_code,
            "未知 MoveIt 错误",
        )

        # 如果错误码表示成功，则输出成功日志。
        if error_code == MoveItErrorCodes.SUCCESS:
            # 输出完整位姿目标执行成功。
            self.get_logger().info(
                "✅ MoveIt 已成功执行完整 6D 位姿目标"
            )

        # 如果错误码不是成功，则输出错误码与中文原因。
        else:
            # 输出失败原因。
            self.get_logger().error(
                f"MoveIt 未完成完整位姿目标 | "
                f"code={error_code}，原因：{error_message}"
            )

        # 标记任务已经结束。
        self.goal_finished = True


# 定义 ROS 2 命令行入口函数。
def main(args=None):
    # 初始化 ROS 2 通信环境。
    rclpy.init(args=args)

    # 创建完整位姿客户端节点。
    node = MoveItPoseClient()

    # 在任务未完成时持续处理定时器、反馈和结果。
    while rclpy.ok() and not node.goal_finished:
        # 每次最多等待 0.1 秒。
        rclpy.spin_once(node, timeout_sec=0.1)

    # 销毁节点，释放资源。
    node.destroy_node()

    # 关闭 ROS 2 通信环境。
    rclpy.shutdown()


# 当文件被直接执行时，调用 main 函数。
if __name__ == "__main__":
    main()