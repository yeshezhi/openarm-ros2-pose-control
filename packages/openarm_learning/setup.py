import os
from glob import glob

from setuptools import find_packages, setup

package_name = "openarm_learning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cjy",
    maintainer_email="cjy@todo.todo",
    description="ROS 2 learning nodes and demos for OpenArm.",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    # 配置 ros2 run 可执行节点。
    entry_points={
        # 声明控制台脚本列表。
        "console_scripts": [
            # 注册关节角度监测节点。
            "joint_state_monitor = openarm_learning.joint_state_monitor:main",
            # 注册姿态发布节点。
            "pose_publisher = openarm_learning.pose_publisher:main",
            # 注册末端位姿监测节点。
            "end_effector_monitor = openarm_learning.end_effector_monitor:main",
            # 注册目标到达检测器。
            "target_reach_monitor = openarm_learning.target_reach_monitor:main",
            # 注册右臂轨迹 Action 客户端。
            "right_arm_trajectory_client = "
            "openarm_learning.right_arm_trajectory_client:main",
                        # 注册末端位置到 MoveIt 的规划客户端。
            "moveit_position_client = "
            "openarm_learning.moveit_position_client:main",
                        # 注册完整 6D 末端位姿 MoveIt 客户端。
            "moveit_pose_client = openarm_learning.moveit_pose_client:main",
            # 注册目标位姿监听节点，使其可以被 ros2 run 启动。
            "pose_goal_listener = openarm_learning.pose_goal_listener:main",
            # 注册“话题目标位姿到 MoveIt”的执行节点。
            "pose_goal_moveit_client = openarm_learning.pose_goal_moveit_client:main",
            # 注册自动判定目标是否到达的节点。
            "pose_goal_reach_monitor = openarm_learning.pose_goal_reach_monitor:main",
            # 注册规划场景障碍物发布节点。
            "planning_scene_obstacle = openarm_learning.planning_scene_obstacle:main",
            # 注册支持连续目标队列的 MoveIt 客户端。
            "queued_pose_moveit_client = openarm_learning.queued_pose_moveit_client:main",

            "execute_pose_action_server = openarm_learning.execute_pose_action_server:main",

            "queued_execute_pose_action_server = openarm_learning.queued_execute_pose_action_server:main",

        ],
    },
)