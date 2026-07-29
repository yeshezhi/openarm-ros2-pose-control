# 导入 ROS 2 Launch 描述类。
from launch import LaunchDescription

# 导入用于启动 ROS 2 节点的 Node 动作。
from launch_ros.actions import Node


# 定义 Launch 文件入口函数。
def generate_launch_description():
    # 创建连续目标队列执行节点。
    queued_pose_moveit_client = Node(
        # 指定节点所在软件包。
        package='openarm_learning',
        # 指定队列客户端可执行文件。
        executable='queued_pose_moveit_client',
        # 指定节点名称。
        name='queued_pose_moveit_client',
        # 将日志输出到终端。
        output='screen',
    )

    # 创建现有的 TF 自动到达判定节点。
    pose_goal_reach_monitor = Node(
        # 指定节点所在软件包。
        package='openarm_learning',
        # 指定到达判定器可执行文件。
        executable='pose_goal_reach_monitor',
        # 指定节点名称。
        name='pose_goal_reach_monitor',
        # 将日志输出到终端。
        output='screen',
    )

    # 同时启动队列执行器和到达判定器。
    return LaunchDescription([
        # 启动队列执行器。
        queued_pose_moveit_client,
        # 启动到达判定器。
        pose_goal_reach_monitor,
    ])