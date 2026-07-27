import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory(
        "openarm_description"
    )

    xacro_path = os.path.join(
        description_share,
        "assets",
        "robot",
        "openarm_v2.0",
        "urdf",
        "openarm_v20.urdf.xacro",
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "robot_preset": "right_arm",
            "collapse_internal_empty_links": "true",
            "emit_grasp_frame": "false",
        },
    ).toxml()

    rviz_config = os.path.join(
        description_share,
        "rviz",
        "arm_only.rviz",
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
        ),
        Node(
            package="openarm_learning",
            executable="pose_publisher",
            name="pose_publisher",
            output="screen",
        ),
    ])