import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import tf_transformations  # 用于欧拉角和四元数之间的转换
import math
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

class ComputePathClient(Node):

    def __init__(self):
        super().__init__("compute_path_client")
        self.pose_publisher = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.current_pose = None

        # 订阅 odom 以获取当前位置
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # 创建 tf2 缓存和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def odom_callback(self, msg):
        try:
            # 获取 odom 到 map 的转换
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())

            # 创建 PoseStamped 对象
            # map_pose = Odometry()
            odom_pose = PoseStamped()
            odom_pose.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
            odom_pose.header.frame_id = 'odom' # 设置参考坐标系
            odom_pose.pose.position = msg.pose.pose.position
            odom_pose.pose.orientation = msg.pose.pose.orientation
            # 使用转换将 odom_pose 转换为 map 坐标系
            # map_pose = do_transform_pose(odom_pose, transform)
            point_in_map = self.tf_buffer.transform(odom_pose, 'map')
            
            
            # 更新当前位置为 map 坐标系中的位姿
            self.current_pose = point_in_map.pose
        except TransformException as ex:
            self.get_logger().warn(f"Transform from odom to map failed: {ex}")
 

    def create_pose(self, x, y, z, yaw):
        # 创建 PoseStamped 对象并设置位置
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position = Point(x=x, y=y, z=z)

        # 将 yaw 转换为四元数
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

        # 设置四元数到姿态的 orientation
        pose.pose.orientation = Quaternion(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
        )
        return pose

    def send_goal(self, pose):
        # 发布目标点
        self.pose_publisher.publish(pose)
        self.get_logger().info(f"Published goal: {pose.pose.position.x}, {pose.pose.position.y}")

    def is_near_goal(self, goal_pose, tolerance=3.0):
        if self.current_pose is None:
            return False

        # 计算当前坐标与目标点之间的距离
        dx = goal_pose.pose.position.x - self.current_pose.position.x
        dy = goal_pose.pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        self.get_logger().info(f"Distance to goal: {distance}")
        return distance < tolerance  # 如果距离小于容忍范围，认为到达目标


def main(args=None):
    rclpy.init(args=args)
    node = ComputePathClient()

    # 定义三个目标点
    single_pose_list = [
        #{'x': 1.3, 'y': -5.8, 'z': 0.0, 'yaw': -3.4},  # 第三个目标点，yaw=180度
        #{'x': 10.89, 'y': -1.56, 'z': 0.0, 'yaw':-3.07},  # 第一个目标点
        #{'x': -2.07, 'y': -1.15, 'z': 0.0, 'yaw': -3.14}  # 第二个目标点，yaw=90度

        # Point 1 直道中央开始
        {'x': 0.0146346, 'y': -5.88931,'z': 0.0, 'yaw': -1.58907},
        # Point 2 右下角弯 1号 弯心
        {'x': 0.379164, 'y': -13.9076, 'z': 0.0, 'yaw': -0.503171},
            # [rviz2]: Setting goal pose: Frame:map, Position(0.433538, -13.4188, 0), Orientation(0, 0, -0.420325, 0.907374) = Angle: -0.867606
            # Setting goal pose: Frame:map, Position(0.379164, -13.7076, 0), Orientation(0, 0, -0.24894, 0.968519) = Angle: -0.503171
        # Point 3 右上角弯 2号 进弯
        {'x': 7.24757, 'y': -14.6056,  'z': 0.0, 'yaw': 1.13497},
        # Point 4 右上角弯 2号 弯心
        {'x': 7.14134, 'y': -11.5419,  'z': 0.0, 'yaw': 2.36201},
        # Point 5 上方直线点
        {'x': 4.15308, 'y': -7.53755,  'z': 0.0, 'yaw': 1.53068},
        # {'x': 4.35308, 'y': -8.03755,  'z': 0.0, 'yaw': 1.53068},# speed
        # Point 6 上方弯  3号 左上方直道
        {'x': 5.98572, 'y': -3.86257,  'z': 0.0, 'yaw': 0.969711},
            # Setting goal pose: Frame:map, Position(5.88572, -3.86257, 0), Orientation(0, 0, 0.466081, 0.884742) = Angle: 0.969711
        # Point 7 左上角弯 4号 出弯带斜率
        {'x': 8.21546, 'y': 0.9646,  'z': 0.0, 'yaw': 2.48867},
        # Point 8 左下角弯 5号 直线出弯
        {'x': 4.08559, 'y': 2.62511,  'z': 0.0, 'yaw': -2.61212},
        {'x': 0.744836, 'y': -1.19677,  'z': 0.0, 'yaw': -1.77693}
            # 修改为两个
            #  Setting goal pose: Frame:map, Position(4.88559, 2.52511, 0), Orientation(0, 0, -0.965162, 0.261653) = Angle: -2.61212
        #  Setting goal pose: Frame:map, Position(0.844836, -1.19677, 0), Orientation(0, 0, -0.776105, 0.630603) = Angle: -1.77693
        
    ]

    for i in range(5):# 5 loops
        for pose_dict in single_pose_list:
            # 创建目标点
            pose = node.create_pose(
                x=pose_dict['x'],
                y=pose_dict['y'],
                z=pose_dict['z'],
                yaw=pose_dict['yaw']
            )
            
            # 发送目标点
            node.send_goal(pose)

            # 等待机器人接近目标点
            while not node.is_near_goal(pose):
                rclpy.spin_once(node, timeout_sec=0.1)  # 不断轮询当前位置，检测是否接近目标

    rclpy.shutdown()


if __name__ == "__main__":
    main()
