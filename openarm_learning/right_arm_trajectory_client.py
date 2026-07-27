# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入 Action 客户端类，用于向控制器发送可执行任务。
from rclpy.action import ActionClient

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入官方轨迹控制 Action 类型。
from control_msgs.action import FollowJointTrajectory

# 导入单个关节轨迹点消息类型。
from trajectory_msgs.msg import JointTrajectoryPoint

# 导入时间长度消息类型。
from builtin_interfaces.msg import Duration


# 定义右臂七个关节的标准顺序。
RIGHT_ARM_JOINTS = [
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
]

# 定义 home 姿态的七个关节角，单位为弧度。
HOME_POSITIONS = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
]


# 定义一个节点，用于向官方右臂轨迹控制器发送轨迹。
class RightArmTrajectoryClient(Node):

    # 初始化节点和 Action 客户端。
    def __init__(self):
        # 调用父类构造函数，并设置节点名称。
        super().__init__("right_arm_trajectory_client")

        # 创建 Action 客户端，并连接官方右臂轨迹控制器。
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/right_joint_trajectory_controller/follow_joint_trajectory",
        )

        # 记录任务是否完成，供 main 函数决定何时退出。
        self.goal_finished = False

    # 创建并发送“回到 home”的关节轨迹。
    def send_home_goal(self):
        # 等待控制器 Action 服务端最多 5 秒。
        server_ready = self.action_client.wait_for_server(timeout_sec=5.0)

        # 如果控制器未启动，则打印错误并结束任务。
        if not server_ready:
            # 输出错误信息。
            self.get_logger().error("未找到右臂轨迹控制器 Action 服务端")

            # 标记任务结束。
            self.goal_finished = True

            # 结束当前函数。
            return

        # 创建 FollowJointTrajectory 的目标对象。
        goal = FollowJointTrajectory.Goal()

        # 指定这段轨迹控制的七个右臂关节。
        goal.trajectory.joint_names = RIGHT_ARM_JOINTS

        # 创建一个轨迹点，表示轨迹终点。
        point = JointTrajectoryPoint()

        # 设置终点处七个关节的目标位置。
        point.positions = HOME_POSITIONS

        # 设置从当前姿态运动到 home 姿态需要 3 秒。
        point.time_from_start = Duration(sec=3, nanosec=0)

        # 将这个终点加入轨迹点列表。
        goal.trajectory.points = [point]

        # 打印即将发送的目标信息。
        self.get_logger().info("发送右臂 home 轨迹，预计执行时间：3 秒")

        # 异步发送目标，并注册反馈回调函数。
        send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        # 当控制器接受或拒绝目标时，调用 goal_response_callback。
        send_goal_future.add_done_callback(self.goal_response_callback)

    # 处理控制器对目标的接受或拒绝。
    def goal_response_callback(self, future):
        # 取出控制器返回的目标句柄。
        goal_handle = future.result()

        # 如果控制器拒绝该轨迹，则打印错误并结束。
        if not goal_handle.accepted:
            # 输出拒绝信息。
            self.get_logger().error("轨迹目标被控制器拒绝")

            # 标记任务结束。
            self.goal_finished = True

            # 结束当前函数。
            return

        # 输出控制器接受目标的提示。
        self.get_logger().info("轨迹目标已被控制器接受，开始执行")

        # 异步等待最终执行结果。
        result_future = goal_handle.get_result_async()

        # 当轨迹结束时，调用 result_callback。
        result_future.add_done_callback(self.result_callback)

    # 处理控制器执行过程中的实时反馈。
    def feedback_callback(self, feedback_message):
        # 取出 Action 反馈中的具体反馈内容。
        feedback = feedback_message.feedback

        # 读取期望轨迹中第四关节的当前位置。
        desired_joint4 = feedback.desired.positions[3]

        # 读取实际反馈中第四关节的当前位置。
        actual_joint4 = feedback.actual.positions[3]

        # 输出第四关节的期望值与实际值，单位为弧度。
        self.get_logger().info(
            "执行反馈 | "
            f"joint4 期望={desired_joint4:.3f} rad，"
            f"实际={actual_joint4:.3f} rad"
        )

    # 处理轨迹执行结束后的最终结果。
    def result_callback(self, future):
        # 取出完整的 Action 返回结果。
        action_result = future.result()

        # 取出 FollowJointTrajectory 的结果部分。
        result = action_result.result

        # 判断错误码是否为 0，也就是成功。
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            # 输出成功信息。
            self.get_logger().info("✅ 右臂已成功到达 home 姿态")

        # 如果错误码不是 0，则输出失败原因。
        else:
            # 输出错误码与控制器给出的文字说明。
            self.get_logger().error(
                f"轨迹执行失败 | code={result.error_code}，"
                f"reason={result.error_string}"
            )

        # 标记任务已经结束。
        self.goal_finished = True


# 定义 ROS 2 命令行入口函数。
def main(args=None):
    # 初始化 ROS 2 通信环境。
    rclpy.init(args=args)

    # 创建右臂轨迹客户端节点。
    node = RightArmTrajectoryClient()

    # 向控制器发送 home 轨迹。
    node.send_home_goal()

    # 在任务未完成时持续处理 Action 返回和反馈。
    while rclpy.ok() and not node.goal_finished:
        # 每次最多等待 0.1 秒，以便持续接收反馈。
        rclpy.spin_once(node, timeout_sec=0.1)

    # 销毁节点，释放资源。
    node.destroy_node()

    # 关闭 ROS 2 通信环境。
    rclpy.shutdown()


# 当文件被直接执行时，调用 main 函数。
if __name__ == "__main__":
    main()