# 导入 Python 的数学库，用于三角函数和弧度转角度。
import math

# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入 ROS 2 的时间类型；Time() 表示查询最新 TF。
from rclpy.time import Time

# 导入 TF 缓冲区、监听器和异常类型。
from tf2_ros import Buffer, TransformException, TransformListener


# 定义一个函数：把四元数转换为 Roll、Pitch、Yaw。
def quaternion_to_rpy_degrees(qx, qy, qz, qw):
    # 计算绕 X 轴旋转角 Roll 的正弦分子。
    sin_roll = 2.0 * (qw * qx + qy * qz)

    # 计算绕 X 轴旋转角 Roll 的余弦分母。
    cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy)

    # 使用 atan2 计算 Roll，结果单位为弧度。
    roll = math.atan2(sin_roll, cos_roll)

    # 计算绕 Y 轴旋转角 Pitch 的正弦值。
    sin_pitch = 2.0 * (qw * qy - qz * qx)

    # 判断是否接近 Pitch 的极限位置，避免 asin 的输入超出范围。
    if abs(sin_pitch) >= 1.0:
        # 接近极限时，把 Pitch 设为正或负 90 度。
        pitch = math.copysign(math.pi / 2.0, sin_pitch)

    # 一般情况下直接使用 asin 计算 Pitch。
    else:
        # 计算 Pitch，结果单位为弧度。
        pitch = math.asin(sin_pitch)

    # 计算绕 Z 轴旋转角 Yaw 的正弦分子。
    sin_yaw = 2.0 * (qw * qz + qx * qy)

    # 计算绕 Z 轴旋转角 Yaw 的余弦分母。
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)

    # 使用 atan2 计算 Yaw，结果单位为弧度。
    yaw = math.atan2(sin_yaw, cos_yaw)

    # 将三个弧度角转换为度，并作为一个元组返回。
    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


# 定义末端位姿监测节点。
class EndEffectorMonitor(Node):

    # 初始化节点、TF 工具和定时器。
    def __init__(self):
        # 调用父类构造函数，并设定 ROS 节点名称。
        super().__init__("end_effector_monitor")

        # 创建 TF 缓冲区，用来保存收到的坐标变换数据。
        self.tf_buffer = Buffer()

        # 创建 TF 监听器；它会订阅 /tf 和 /tf_static。
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 每隔 0.5 秒查询一次末端位姿。
        self.timer = self.create_timer(0.5, self.print_end_effector_pose)

        # 打印节点启动提示。
        self.get_logger().info("正在监听 world → openarm_right_link7 的完整位姿")

    # 查询并打印末端的位置、四元数和 RPY 姿态角。
    def print_end_effector_pose(self):
        # 使用 try 防止 TF 暂未准备好时让节点退出。
        try:
            # 查询 world 到 openarm_right_link7 的最新坐标变换。
            transform = self.tf_buffer.lookup_transform(
                "world",
                "openarm_right_link7",
                Time(),
            )

            # 取出末端的位置 xyz。
            position = transform.transform.translation

            # 取出末端的四元数朝向 xyzw。
            orientation = transform.transform.rotation

            # 调用转换函数，得到单位为度的 Roll、Pitch、Yaw。
            roll, pitch, yaw = quaternion_to_rpy_degrees(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )

            # 输出末端位置，单位为米。
            self.get_logger().info(
                "位置 | "
                f"x={position.x:.3f} m, "
                f"y={position.y:.3f} m, "
                f"z={position.z:.3f} m"
            )

            # 输出四元数；它无单位，主要用于程序内部计算。
            self.get_logger().info(
                "四元数 | "
                f"qx={orientation.x:.3f}, "
                f"qy={orientation.y:.3f}, "
                f"qz={orientation.z:.3f}, "
                f"qw={orientation.w:.3f}"
            )

            # 输出转换后的姿态角，单位为度，便于人理解。
            self.get_logger().info(
                "RPY姿态 | "
                f"roll={roll:.1f}°, "
                f"pitch={pitch:.1f}°, "
                f"yaw={yaw:.1f}°"
            )

        # TF 查询失败时打印警告，但保持节点继续运行。
        except TransformException as error:
            # 输出错误信息，等待下一次定时器重新查询。
            self.get_logger().warning(f"暂时无法获取末端 TF: {error}")


# 定义 ROS 2 命令行入口函数。
def main(args=None):
    # 初始化 ROS 2 通信环境。
    rclpy.init(args=args)

    # 创建末端位姿监测节点。
    node = EndEffectorMonitor()

    # 持续运行节点。
    try:
        rclpy.spin(node)

    # 用户按 Ctrl+C 时正常退出。
    except KeyboardInterrupt:
        pass

    # 退出前释放节点与 ROS 2 资源。
    finally:
        node.destroy_node()
        rclpy.shutdown()


# 当该文件被直接执行时，调用 main 函数。
if __name__ == "__main__":
    main()