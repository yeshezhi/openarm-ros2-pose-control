# 导入 ROS 2 的 Python 客户端库。
import rclpy

# 导入 ROS 2 节点基类。
from rclpy.node import Node

# 导入碰撞物体的位姿消息。
from geometry_msgs.msg import Pose

# 导入 MoveIt 的碰撞物体消息。
from moveit_msgs.msg import CollisionObject

# 导入 MoveIt 的规划场景消息。
from moveit_msgs.msg import PlanningScene

# 导入 MoveIt 的“应用规划场景”服务类型。
from moveit_msgs.srv import ApplyPlanningScene

# 导入盒子等基本几何体消息。
from shape_msgs.msg import SolidPrimitive


# 定义障碍物所在的参考坐标系。
REFERENCE_FRAME = 'world'

# 定义障碍物唯一名称，添加与删除都使用该名称。
OBSTACLE_ID = 'demo_box'


# 定义规划场景障碍物管理节点。
class PlanningSceneObstacle(Node):

    # 初始化节点。
    def __init__(self):
        # 创建 ROS 2 节点。
        super().__init__('planning_scene_obstacle')

        # 声明障碍物中心的 X 坐标参数，单位为米。
        self.declare_parameter('x', 0.12)

        # 声明障碍物中心的 Y 坐标参数，单位为米。
        self.declare_parameter('y', -0.154)

        # 声明障碍物中心的 Z 坐标参数，单位为米。
        self.declare_parameter('z', 0.262)

        # 声明是否执行删除操作的参数。
        self.declare_parameter('remove', False)

        # 读取障碍物中心的 X 坐标。
        self.x = self.get_parameter('x').value

        # 读取障碍物中心的 Y 坐标。
        self.y = self.get_parameter('y').value

        # 读取障碍物中心的 Z 坐标。
        self.z = self.get_parameter('z').value

        # 读取当前是否为删除模式。
        self.remove = self.get_parameter('remove').value

        # 创建 MoveIt 规划场景服务客户端。
        self.apply_scene_client = self.create_client(
            # 指定服务类型。
            ApplyPlanningScene,
            # 指定 MoveIt 提供的服务名称。
            '/apply_planning_scene',
        )

        # 每秒尝试一次，直到成功调用 MoveIt 服务。
        self.timer = self.create_timer(1.0, self.apply_scene_once)

        # 记录是否已经发出服务请求。
        self.has_sent_request = False

        # 输出节点启动提示。
        self.get_logger().info('等待 MoveIt 的 /apply_planning_scene 服务')

    # 构建场景更新并调用 MoveIt 服务。
    def apply_scene_once(self):
        # 如果已经发送过服务请求，则不再重复发送。
        if self.has_sent_request:
            # 结束本次回调。
            return

        # 若 MoveIt 服务尚未启动，则继续等待。
        if not self.apply_scene_client.wait_for_service(timeout_sec=0.0):
            # 输出等待提示。
            self.get_logger().info('MoveIt 规划场景服务尚未就绪，继续等待')

            # 结束本次回调。
            return

        # 创建规划场景差异更新消息。
        planning_scene = PlanningScene()

        # 标记为差异更新，表示只改动部分场景内容。
        planning_scene.is_diff = True

        # 创建碰撞物体消息。
        collision_object = CollisionObject()

        # 指定碰撞物体所在坐标系。
        collision_object.header.frame_id = REFERENCE_FRAME

        # 指定碰撞物体唯一名称。
        collision_object.id = OBSTACLE_ID

        # 如果当前处于删除模式。
        if self.remove:
            # 指定本次操作是删除同名物体。
            collision_object.operation = CollisionObject.REMOVE

            # 输出删除提示。
            self.get_logger().info(f'请求删除障碍物：{OBSTACLE_ID}')

        # 如果当前处于添加模式。
        else:
            # 创建一个长方体障碍物。
            box = SolidPrimitive()

            # 指定几何体为盒子。
            box.type = SolidPrimitive.BOX

            # 设置较薄的 X 方向盒子，避免初始位置与障碍物重叠。
            box.dimensions = [0.01, 0.10, 0.10]

            # 创建盒子在 world 坐标系中的位姿。
            box_pose = Pose()

            # 设置盒子中心的 X 坐标。
            box_pose.position.x = float(self.x)

            # 设置盒子中心的 Y 坐标。
            box_pose.position.y = float(self.y)

            # 设置盒子中心的 Z 坐标。
            box_pose.position.z = float(self.z)

            # 使用单位四元数，表示盒子不旋转。
            box_pose.orientation.w = 1.0

            # 将盒子几何体加入碰撞物体。
            collision_object.primitives.append(box)

            # 将盒子位姿加入碰撞物体。
            collision_object.primitive_poses.append(box_pose)

            # 指定本次操作是添加物体。
            collision_object.operation = CollisionObject.ADD

            # 输出添加提示。
            self.get_logger().info(
                f'请求添加障碍物 | id={OBSTACLE_ID} | '
                f'x={self.x:.3f} m, y={self.y:.3f} m, z={self.z:.3f} m'
            )

        # 将碰撞物体加入规划场景世界模型。
        planning_scene.world.collision_objects.append(collision_object)

        # 创建服务请求对象。
        request = ApplyPlanningScene.Request()

        # 将场景更新放入服务请求。
        request.scene = planning_scene

        # 异步调用 MoveIt 服务。
        future = self.apply_scene_client.call_async(request)

        # 服务返回后调用 response_callback 处理结果。
        future.add_done_callback(self.response_callback)

        # 标记服务请求已发出。
        self.has_sent_request = True

    # 处理 MoveIt 服务返回结果。
    def response_callback(self, future):
        # 读取服务响应。
        response = future.result()

        # 若 MoveIt 明确确认场景已应用。
        if response.success:
            # 输出成功提示。
            self.get_logger().info('✅ MoveIt 已确认规划场景更新成功')

        # 若 MoveIt 未能应用场景更新。
        else:
            # 输出失败提示。
            self.get_logger().error('MoveIt 未能应用规划场景更新')


# 定义程序入口函数。
def main():
    # 初始化 ROS 2 通信。
    rclpy.init()

    # 创建障碍物管理节点。
    node = PlanningSceneObstacle()

    # 让节点持续运行并处理服务响应。
    rclpy.spin(node)

    # 节点结束时释放资源。
    node.destroy_node()

    # 关闭 ROS 2 通信。
    rclpy.shutdown()


# 仅当该文件被直接运行时执行。
if __name__ == '__main__':
    # 启动程序入口。
    main()