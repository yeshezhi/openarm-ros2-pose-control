# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入 Action 客户端类，用于调用 move_group 的规划 Action。
from rclpy.action import ActionClient

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入 Pose 消息，用于描述球形目标区域的中心位姿。
from geometry_msgs.msg import Pose

# 导入 MoveIt 的规划并执行 Action。
from moveit_msgs.action import MoveGroup

# 导入 MoveIt 的约束和错误码消息。
from moveit_msgs.msg import Constraints, MoveItErrorCodes, PositionConstraint

# 导入球体几何体消息，用于定义允许到达的空间区域。
from shape_msgs.msg import SolidPrimitive


# 定义 MoveIt 的规划组名称。
PLANNING_GROUP = "right_arm"

# 定义 MoveIt 右臂末端参考 link。
END_EFFECTOR_LINK = "openarm_right_ee_base_link"

# 定义目标点所在的参考坐标系。
REFERENCE_FRAME = "world"

# 定义目标位置的 X 坐标，单位为米。
TARGET_X = 0.030

# 定义目标位置的 Y 坐标，单位为米。
TARGET_Y = -0.154

# 定义目标位置的 Z 坐标，单位为米。
TARGET_Z = 0.262

# 定义目标球半径，0.015 米就是 1.5 厘米。
POSITION_TOLERANCE = 0.015


# 定义一个节点，用于向 MoveIt 发送右臂末端位置目标。
class MoveItPositionClient(Node):

    # 初始化节点和 MoveIt Action 客户端。
    def __init__(self):
        # 调用父类构造函数，并设定节点名称。
        super().__init__("moveit_position_client")

        # 创建客户端，并连接 move_group 提供的 /move_action。
        self.action_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )

        # 记录任务是否结束，供 main 函数决定何时退出。
        self.goal_finished = False

        # 记录上一条 MoveIt 状态，避免重复刷屏。
        self.last_feedback_state = ""

    # 创建并发送末端位置规划请求。
    def send_position_goal(self):
        # 等待 MoveIt 的 /move_action 服务端最多 5 秒。
        server_ready = self.action_client.wait_for_server(timeout_sec=5.0)

        # 若 move_group 未启动，则输出错误并结束。
        if not server_ready:
            # 输出错误信息。
            self.get_logger().error("未找到 MoveIt 的 /move_action 服务端")

            # 标记任务结束。
            self.goal_finished = True

            # 结束当前函数。
            return

        # 创建 MoveGroup 的目标对象。
        goal = MoveGroup.Goal()

        # 指定这次规划使用右臂规划组。
        goal.request.group_name = PLANNING_GROUP

        # 让 MoveIt 从当前真实关节状态开始规划。
        goal.request.start_state.is_diff = True

        # 设置最多尝试 5 次规划。
        goal.request.num_planning_attempts = 5

        # 设置每次规划最多可用 5 秒。
        goal.request.allowed_planning_time = 5.0

        # 设置最大速度缩放为 10%，便于安全观察。
        goal.request.max_velocity_scaling_factor = 0.10

        # 设置最大加速度缩放为 10%，便于安全观察。
        goal.request.max_acceleration_scaling_factor = 0.10

        # 创建末端位置约束对象。
        position_constraint = PositionConstraint()

        # 指定目标点使用 world 坐标系。
        position_constraint.header.frame_id = REFERENCE_FRAME

        # 指定被约束的是右臂末端参考 link。
        position_constraint.link_name = END_EFFECTOR_LINK

        # 创建一个球体，作为允许末端进入的目标区域。
        target_sphere = SolidPrimitive()

        # 指定该几何体是球体。
        target_sphere.type = SolidPrimitive.SPHERE

        # 设置球体半径为 1.5 厘米。
        target_sphere.dimensions = [POSITION_TOLERANCE]

        # 创建球体中心的位姿。
        target_pose = Pose()

        # 设置球体中心的 X 坐标。
        target_pose.position.x = TARGET_X

        # 设置球体中心的 Y 坐标。
        target_pose.position.y = TARGET_Y

        # 设置球体中心的 Z 坐标。
        target_pose.position.z = TARGET_Z

        # 设置单位四元数，表示球体本身不需要旋转。
        target_pose.orientation.w = 1.0

        # 将球体放入位置约束区域。
        position_constraint.constraint_region.primitives = [target_sphere]

        # 将球体中心位姿放入位置约束区域。
        position_constraint.constraint_region.primitive_poses = [target_pose]

        # 设置该位置约束的权重为 1。
        position_constraint.weight = 1.0

        # 创建一组目标约束。
        goal_constraints = Constraints()

        # 将末端位置约束放入目标约束组。
        goal_constraints.position_constraints = [position_constraint]

        # 将这一组约束设置为 MoveIt 的目标。
        goal.request.goal_constraints = [goal_constraints]

        # 设置为 false，表示规划成功后直接执行。
        goal.planning_options.plan_only = False

        # 允许发生环境变化时尝试重新规划。
        goal.planning_options.replan = True

        # 最多允许重新规划两次。
        goal.planning_options.replan_attempts = 2

        # 标记规划场景是增量更新。
        goal.planning_options.planning_scene_diff.is_diff = True

        # 标记机器人状态从当前状态增量读取。
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        # 输出即将发送的目标位置。
        self.get_logger().info(
            "发送 MoveIt 末端位置目标 | "
            f"x={TARGET_X:.3f} m, "
            f"y={TARGET_Y:.3f} m, "
            f"z={TARGET_Z:.3f} m"
        )

        # 异步发送目标，并注册反馈处理函数。
        send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        # 注册目标接受或拒绝的回调函数。
        send_goal_future.add_done_callback(self.goal_response_callback)

    # 处理 MoveIt 对目标的接受或拒绝。
    def goal_response_callback(self, future):
        # 取出 MoveIt 返回的目标句柄。
        goal_handle = future.result()

        # 若 MoveIt 拒绝目标，则输出错误并结束。
        if not goal_handle.accepted:
            # 输出拒绝信息。
            self.get_logger().error("MoveIt 拒绝了该末端位置目标")

            # 标记任务结束。
            self.goal_finished = True

            # 结束当前函数。
            return

        # 输出 MoveIt 已接受请求的信息。
        self.get_logger().info("MoveIt 已接受请求，正在进行 IK、规划与执行")

        # 异步等待规划和执行的最终结果。
        result_future = goal_handle.get_result_async()

        # 注册最终结果回调函数。
        result_future.add_done_callback(self.result_callback)

    # 处理 MoveIt 在规划和执行过程中返回的状态。
    def feedback_callback(self, feedback_message):
        # 读取 MoveIt 的状态文字。
        state = feedback_message.feedback.state

        # 只在状态发生变化时打印信息。
        if state != self.last_feedback_state:
            # 输出当前 MoveIt 状态。
            self.get_logger().info(f"MoveIt 状态 | {state}")

            # 记录最新状态。
            self.last_feedback_state = state

    # 处理规划与执行结束后的结果。
    def result_callback(self, future):
        # 取出 Action 返回的完整结果。
        action_result = future.result()

        # 取出 MoveIt 的规划结果。
        result = action_result.result

        # 判断 MoveIt 是否返回成功码。
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            # 输出规划和执行成功信息。
            self.get_logger().info("✅ MoveIt 已成功规划并执行末端位置目标")

        # 若失败，则输出错误码。
        else:
            # 输出失败错误码，便于后续定位原因。
            self.get_logger().error(
                f"MoveIt 未完成目标 | error_code={result.error_code.val}"
            )

        # 标记任务结束。
        self.goal_finished = True


# 定义 ROS 2 命令行入口函数。
def main(args=None):
    # 初始化 ROS 2 通信环境。
    rclpy.init(args=args)

    # 创建 MoveIt 末端位置客户端节点。
    node = MoveItPositionClient()

    # 发送末端位置规划请求。
    node.send_position_goal()

    # 在任务未完成时持续处理 Action 的反馈和结果。
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