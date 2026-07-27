import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


LIMITS_DEG = {
    1: (-80.0, 200.0),
    2: (-10.0, 190.0),
    3: (-90.0, 90.0),
    4: (0.0, 140.0),
    5: (-90.0, 90.0),
    6: (-45.0, 45.0),
    7: (-90.0, 90.0),
}

WARNING_MARGIN_DEG = 5.0


class JointStateMonitor(Node):
    def __init__(self):
        super().__init__("joint_state_monitor")
        self.last_print_time = 0.0

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.get_logger().info("正在监听 /joint_states，并检查关节限位")

    def joint_state_callback(self, msg: JointState):
        now = time.monotonic()
        if now - self.last_print_time < 1.0:
            return

        positions = dict(zip(msg.name, msg.position))
        angles = []
        warnings = []

        for index in range(1, 8):
            name = f"openarm_right_joint{index}"
            if name not in positions:
                continue

            degree = math.degrees(positions[name])
            lower, upper = LIMITS_DEG[index]
            angles.append(f"joint{index}: {degree:7.2f}°")

            if degree <= lower + WARNING_MARGIN_DEG:
                warnings.append(
                    f"joint{index} 接近下限 {lower:.0f}°"
                )
            elif degree >= upper - WARNING_MARGIN_DEG:
                warnings.append(
                    f"joint{index} 接近上限 {upper:.0f}°"
                )

        self.get_logger().info(" | ".join(angles))

        if warnings:
            self.get_logger().warning("；".join(warnings))

        self.last_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()