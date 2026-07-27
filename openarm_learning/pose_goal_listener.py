# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入带坐标系和时间戳的完整位姿消息类型。
from geometry_msgs.msg import PoseStamped


# 定义“目标位姿监听器”节点类。
class PoseGoalListener(Node):

    # 初始化节点。
    def __init__(self):
        # 创建节点，并给它取名为 pose_goal_listener。
        super().__init__('pose_goal_listener')

        # 订阅目标位姿话题；收到消息时调用 pose_callback 函数。
        self.subscription = self.create_subscription(
            # 指定该话题传递的消息类型为 PoseStamped。
            PoseStamped,
            # 指定监听的话题名称。
            '/openarm/target_pose',
            # 指定收到消息后的回调函数。
            self.pose_callback,
            # 设置消息队列长度为 10。
            10,
        )

        # 输出启动提示。
        self.get_logger().info('正在等待 /openarm/target_pose 目标位姿消息')

    # 每收到一条目标位姿消息，就自动执行一次这个函数。
    def pose_callback(self, message):
        # 读取消息中的三维位置。
        position = message.pose.position

        # 读取消息中的四元数朝向。
        orientation = message.pose.orientation

        # 输出该目标所使用的坐标系。
        self.get_logger().info(f'收到目标坐标系：{message.header.frame_id}')

        # 输出位置，单位是米。
        self.get_logger().info(
            f'位置 | x={position.x:.3f} m, '
            f'y={position.y:.3f} m, '
            f'z={position.z:.3f} m'
        )

        # 输出朝向；四元数没有单位。
        self.get_logger().info(
            f'朝向 | qx={orientation.x:.3f}, '
            f'qy={orientation.y:.3f}, '
            f'qz={orientation.z:.3f}, '
            f'qw={orientation.w:.3f}'
        )


# 定义程序入口函数。
def main():
    # 初始化 ROS 2 通信。
    rclpy.init()

    # 创建监听器节点对象。
    node = PoseGoalListener()

    # 让节点持续运行并等待话题消息。
    rclpy.spin(node)

    # 节点停止后销毁节点对象。
    node.destroy_node()

    # 关闭 ROS 2 通信。
    rclpy.shutdown()


# 仅在该文件被直接运行时执行 main 函数。
if __name__ == '__main__':
    # 启动程序入口。
    main()