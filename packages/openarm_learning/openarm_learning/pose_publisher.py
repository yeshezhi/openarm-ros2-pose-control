import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PosePublisher(Node):
    def __init__(self):
        super().__init__("pose_publisher")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.joint_names = [
            f"openarm_right_joint{i}"
            for i in range(1, 8)
        ]

        self.poses = [
            (
                "home",
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
            (
                "raised_arm",
                [
                    0.2,
                    math.radians(35),
                    math.radians(-45),
                    math.radians(50),
                    0.0,
                    0.0,
                    0.0,
                ],
            ),
            (
                "reach_forward",
                [
                    0.3,
                    math.radians(50),
                    math.radians(-60),
                    math.radians(80),
                    math.radians(15),
                    0.0,
                    0.0,
                ],
            ),
        ]

        self.segment_duration = 3.0
        self.start_time = self.get_clock().now()
        self.last_target_name = None

        self.timer = self.create_timer(0.05, self.publish_trajectory)
        self.get_logger().info("平滑姿态轨迹已启动")

    def publish_trajectory(self):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        segment = int(elapsed // self.segment_duration)
        progress = (elapsed % self.segment_duration) / self.segment_duration

        start_index = segment % len(self.poses)
        target_index = (start_index + 1) % len(self.poses)

        start_name, start_pose = self.poses[start_index]
        target_name, target_pose = self.poses[target_index]

        # Smoothstep：起点与终点的速度都平滑过渡到 0。
        smooth_progress = progress * progress * (3.0 - 2.0 * progress)

        positions = [
            start + smooth_progress * (target - start)
            for start, target in zip(start_pose, target_pose)
        ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.publisher.publish(msg)

        if target_name != self.last_target_name:
            self.get_logger().info(
                f"开始平滑运动：{start_name} → {target_name}"
            )
            self.last_target_name = target_name


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()