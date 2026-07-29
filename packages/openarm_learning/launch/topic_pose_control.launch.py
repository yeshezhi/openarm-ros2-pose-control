# 导入 ROS 2 Launch 描述类，用于组织多个节点。
from launch import LaunchDescription

# 导入 Node 动作，用于在 Launch 中启动 ROS 2 节点。
from launch_ros.actions import Node


# 定义 Launch 文件入口函数。
def generate_launch_description():
    # 创建“话题目标 → MoveIt → 自动验收”执行节点。
    pose_goal_moveit_client = Node(
        # 指定节点所在的软件包名称。
        package='openarm_learning',
        # 指定要运行的可执行节点名称。
        executable='pose_goal_moveit_client',
        # 指定 ROS 2 节点名称。
        name='pose_goal_moveit_client',
        # 让节点日志直接输出到当前终端。
        output='screen',
    )

    # 创建末端目标到达判定节点。
    pose_goal_reach_monitor = Node(
        # 指定节点所在的软件包名称。
        package='openarm_learning',
        # 指定要运行的可执行节点名称。
        executable='pose_goal_reach_monitor',
        # 指定 ROS 2 节点名称。
        name='pose_goal_reach_monitor',
        # 让节点日志直接输出到当前终端。
        output='screen',
    )

    # 返回 Launch 描述，并同时启动上面两个节点。
    return LaunchDescription([
        # 启动位姿目标执行器。
        pose_goal_moveit_client,
        # 启动自动到达判定器。
        pose_goal_reach_monitor,
    ])