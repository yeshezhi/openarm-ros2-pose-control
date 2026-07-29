# 导入 Python 数学库，用于计算三维直线距离。
import math

# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入 Point 消息类型；它包含 x、y、z 三个坐标。
from geometry_msgs.msg import Point

# 导入 ROS 2 时间类型；Time() 表示查询最新 TF。
from rclpy.time import Time

# 导入 TF 缓冲区、监听器和异常类型。
from tf2_ros import Buffer, TransformException, TransformListener


# 定义到达判定阈值，0.030 米就是 3 厘米。
REACH_THRESHOLD = 0.030


# 定义一个节点，用于接收目标点并判断末端是否到达。
class TargetReachMonitor(Node):

    # 初始化节点、TF 工具、目标点订阅器和定时器。
    def __init__(self):
        # 调用父类构造函数，并设置 ROS 节点名称。
        super().__init__("target_reach_monitor")

        # 创建 TF 缓冲区，用于保存坐标变换信息。
        self.tf_buffer = Buffer()

        # 创建 TF 监听器，用于接收 /tf 和 /tf_static 数据。
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始时还没有收到目标点，因此设置为 None。
        self.target_point = None

        # 初始时默认末端尚未到达目标点。
        self.was_reached = False

        # 订阅目标点话题；收到消息时调用 target_point_callback。
        self.target_subscription = self.create_subscription(
            # 指定消息类型为三维坐标 Point。
            Point,
            # 指定要订阅的话题名称。
            "/openarm/target_point",
            # 指定收到目标点时执行的回调函数。
            self.target_point_callback,
            # 设置消息队列长度为 10。
            10,
        )

        # 每隔 0.5 秒检查一次末端是否已到达目标。
        self.timer = self.create_timer(0.5, self.check_target_distance)

        # 输出启动提示，告知用户该节点正在等待目标点。
        self.get_logger().info("等待 /openarm/target_point 发布目标 xyz")

    # 收到新目标点时执行该函数。
    def target_point_callback(self, message):
        # 保存收到的 Point 消息作为当前目标点。
        self.target_point = message

        # 新目标点到来后，重置上一次的到达状态。
        self.was_reached = False

        # 打印刚刚收到的目标点坐标。
        self.get_logger().info(
            "收到新目标点 | "
            f"x={message.x:.3f} m, "
            f"y={message.y:.3f} m, "
            f"z={message.z:.3f} m"
        )

    # 查询末端位置，并计算它到当前目标点的距离。
    def check_target_distance(self):
        # 如果还没有收到目标点，则不进行距离计算。
        if self.target_point is None:
            # 直接结束本次定时器回调。
            return

        # 使用 try 防止 TF 还未准备好时让节点崩溃。
        try:
            # 查询 world 到右臂末端 link7 的最新坐标变换。
            transform = self.tf_buffer.lookup_transform(
                "world",
                "openarm_right_link7",
                Time(),
            )

            # 取出末端当前位置。
            position = transform.transform.translation

            # 计算当前位置与目标点在 X 方向的差值。
            delta_x = position.x - self.target_point.x

            # 计算当前位置与目标点在 Y 方向的差值。
            delta_y = position.y - self.target_point.y

            # 计算当前位置与目标点在 Z 方向的差值。
            delta_z = position.z - self.target_point.z

            # 根据三维欧氏距离公式计算直线距离。
            distance = math.sqrt(
                delta_x * delta_x
                + delta_y * delta_y
                + delta_z * delta_z
            )

            # 判断距离是否小于或等于 3 厘米阈值。
            reached = distance <= REACH_THRESHOLD

            # 如果刚刚进入目标范围，则输出“已到达”。
            if reached and not self.was_reached:
                # 输出到达提示和当前距离。
                self.get_logger().info(
                    f"✅ 已到达目标点，当前距离为 {distance:.3f} m"
                )

            # 如果刚刚离开目标范围，则输出“离开目标范围”。
            elif not reached and self.was_reached:
                # 输出离开提示和当前距离。
                self.get_logger().info(
                    f"↗️ 已离开目标范围，当前距离为 {distance:.3f} m"
                )

            # 如果尚未到达，则持续输出当前距离。
            elif not reached:
                # 输出当前位置到目标点的距离。
                self.get_logger().info(
                    f"距离当前目标点: {distance:.3f} m"
                )

            # 保存当前到达状态，供下一次检查使用。
            self.was_reached = reached

        # TF 查询失败时输出警告，但节点不会退出。
        except TransformException as error:
            # 输出错误信息，下一次定时器会继续尝试。
            self.get_logger().warning(f"暂时无法获取末端 TF: {error}")


# 定义 ROS 2 命令行入口函数。
def main(args=None):
    # 初始化 ROS 2 通信环境。
    rclpy.init(args=args)

    # 创建目标到达检测节点。
    node = TargetReachMonitor()

    # 让节点持续运行。
    try:
        rclpy.spin(node)

    # 用户按 Ctrl+C 时正常退出。
    except KeyboardInterrupt:
        pass

    # 无论如何退出，都释放节点与 ROS 2 资源。
    finally:
        node.destroy_node()
        rclpy.shutdown()


# 当该文件被直接运行时，调用 main 函数。
if __name__ == "__main__":
    main()