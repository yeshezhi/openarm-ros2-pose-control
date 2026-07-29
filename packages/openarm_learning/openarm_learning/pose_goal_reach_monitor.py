# 导入数学函数，用于计算距离、平方根和反余弦。
import math

# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入用于查询最新 TF 的时间类型。
from rclpy.time import Time

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入带坐标系信息的目标位姿消息。
from geometry_msgs.msg import PoseStamped

# 导入 TF 缓冲区类。
from tf2_ros import Buffer

# 导入 TF 查询失败时的异常类型。
from tf2_ros import TransformException

# 导入 TF 监听器类。
from tf2_ros import TransformListener


# 定义被监测的右臂末端链接名称。
END_EFFECTOR_LINK = 'openarm_right_ee_base_link'

# 定义允许的位置误差，单位为米。
POSITION_TOLERANCE = 0.005

# 使用三个独立轴各 0.15 rad 容差合成后的整体姿态误差上限。
ORIENTATION_TOLERANCE = math.sqrt(3.0) * 0.15


# 定义“目标位姿到达判定器”节点。
class PoseGoalReachMonitor(Node):

    # 初始化节点。
    def __init__(self):
        # 创建 ROS 2 节点。
        super().__init__('pose_goal_reach_monitor')

        # 创建 TF 缓冲区，用于保存坐标变换。
        self.tf_buffer = Buffer()

        # 创建 TF 监听器，让缓冲区持续接收 TF 数据。
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始时还没有收到任何目标位姿。
        self.target_pose = None

        # 用于记录是否已经报告过“到达”。
        self.has_reported_reached = False

        # 订阅目标位姿话题。
        self.subscription = self.create_subscription(
            # 指定话题消息类型。
            PoseStamped,
            # 指定话题名称。
            '/openarm/target_pose',
            # 指定收到目标后的处理函数。
            self.target_callback,
            # 设置消息队列长度。
            10,
        )

        # 每 0.5 秒检查一次末端实际位姿。
        self.timer = self.create_timer(0.5, self.check_reach_status)

        # 输出启动提示。
        self.get_logger().info('正在监听目标位姿，并自动判断右臂末端是否到达')

    # 收到新的目标位姿时执行。
    def target_callback(self, message):
        # 保存最新目标位姿，供定时器持续检查。
        self.target_pose = message

        # 新目标到来后，允许下一次重新报告“已到达”。
        self.has_reported_reached = False

        # 输出收到的目标位置。
        self.get_logger().info(
            f'收到目标 | frame={message.header.frame_id} | '
            f'x={message.pose.position.x:.3f} m, '
            f'y={message.pose.position.y:.3f} m, '
            f'z={message.pose.position.z:.3f} m'
        )

    # 定时查询 TF 并判断是否到达。
    def check_reach_status(self):
        # 若尚未收到目标，则无需计算。
        if self.target_pose is None:
            # 结束本次定时检查。
            return

        # 读取目标使用的参考坐标系。
        reference_frame = self.target_pose.header.frame_id

        # 若发布消息时没有填写坐标系，则无法比较。
        if not reference_frame:
            # 输出错误提示。
            self.get_logger().error('目标消息没有 frame_id，无法判断是否到达')

            # 结束本次定时检查。
            return

        # 尝试查询“参考坐标系 → 右臂末端”的最新 TF。
        try:
            # 获取最新末端实际位姿。
            transform = self.tf_buffer.lookup_transform(
                # 指定目标参考坐标系，例如 world。
                reference_frame,
                # 指定要查询的右臂末端链接。
                END_EFFECTOR_LINK,
                # 使用最新可用的 TF 数据。
                Time(),
            )

        # 若 TF 尚未准备好或坐标系不存在。
        except TransformException as exception:
            # 输出提示信息。
            self.get_logger().warning(f'暂时无法读取末端 TF：{exception}')

            # 结束本次定时检查。
            return

        # 计算 X 方向的位置误差。
        dx = transform.transform.translation.x - self.target_pose.pose.position.x

        # 计算 Y 方向的位置误差。
        dy = transform.transform.translation.y - self.target_pose.pose.position.y

        # 计算 Z 方向的位置误差。
        dz = transform.transform.translation.z - self.target_pose.pose.position.z

        # 计算三维欧氏距离，即总体位置误差。
        position_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        # 读取目标四元数的 X 分量。
        target_qx = self.target_pose.pose.orientation.x

        # 读取目标四元数的 Y 分量。
        target_qy = self.target_pose.pose.orientation.y

        # 读取目标四元数的 Z 分量。
        target_qz = self.target_pose.pose.orientation.z

        # 读取目标四元数的 W 分量。
        target_qw = self.target_pose.pose.orientation.w

        # 读取实际四元数的 X 分量。
        actual_qx = transform.transform.rotation.x

        # 读取实际四元数的 Y 分量。
        actual_qy = transform.transform.rotation.y

        # 读取实际四元数的 Z 分量。
        actual_qz = transform.transform.rotation.z

        # 读取实际四元数的 W 分量。
        actual_qw = transform.transform.rotation.w

        # 计算目标四元数的长度。
        target_norm = math.sqrt(
            target_qx * target_qx +
            target_qy * target_qy +
            target_qz * target_qz +
            target_qw * target_qw
        )

        # 计算实际四元数的长度。
        actual_norm = math.sqrt(
            actual_qx * actual_qx +
            actual_qy * actual_qy +
            actual_qz * actual_qz +
            actual_qw * actual_qw
        )

        # 防止错误四元数长度为零导致除零错误。
        if target_norm == 0.0 or actual_norm == 0.0:
            # 输出错误提示。
            self.get_logger().error('检测到长度为零的四元数，无法计算姿态误差')

            # 结束本次定时检查。
            return

        # 计算两个单位四元数的点积。
        quaternion_dot = (
            target_qx * actual_qx +
            target_qy * actual_qy +
            target_qz * actual_qz +
            target_qw * actual_qw
        ) / (target_norm * actual_norm)

        # 对点积取绝对值，因为 q 和 -q 表示同一空间朝向。
        quaternion_dot = abs(quaternion_dot)

        # 将浮点误差限制到 acos 函数允许的 [-1, 1] 范围。
        quaternion_dot = max(-1.0, min(1.0, quaternion_dot))

        # 根据四元数点积计算两个朝向之间的最小夹角，单位为弧度。
        orientation_error = 2.0 * math.acos(quaternion_dot)

        # 将姿态误差转换为角度，便于观察。
        orientation_error_degrees = math.degrees(orientation_error)

        # 判断位置和朝向是否都满足预设容差。
        reached = (
            position_error <= POSITION_TOLERANCE and
            orientation_error <= ORIENTATION_TOLERANCE
        )

        # 输出当前位置误差与姿态误差。
        self.get_logger().info(
            f'误差 | 位置={position_error * 1000.0:.2f} mm | '
            f'姿态={orientation_error_degrees:.2f}°'
        )

        # 若首次满足到达条件。
        if reached and not self.has_reported_reached:
            # 输出到达成功提示。
            self.get_logger().info('✅ 已到达目标：位置和姿态均在允许容差内')

            # 标记已报告，避免每 0.5 秒重复刷屏。
            self.has_reported_reached = True

        # 若尚未满足到达条件。
        elif not reached:
            # 输出尚未到达提示。
            self.get_logger().info('⏳ 尚未到达目标，或目标不满足容差')

            # 允许后续到达时再次输出成功提示。
            self.has_reported_reached = False


# 定义程序入口函数。
def main():
    # 初始化 ROS 2 通信。
    rclpy.init()

    # 创建到达判定器节点。
    node = PoseGoalReachMonitor()

    # 让节点持续运行。
    rclpy.spin(node)

    # 节点退出时销毁节点。
    node.destroy_node()

    # 关闭 ROS 2 通信。
    rclpy.shutdown()


# 仅在该文件被直接运行时执行。
if __name__ == '__main__':
    # 启动程序入口。
    main()