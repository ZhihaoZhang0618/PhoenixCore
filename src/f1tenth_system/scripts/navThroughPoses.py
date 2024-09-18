import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf_transformations  # 用于欧拉角和四元数之间的转换


class NavigateThroughPosesClient(Node):

    def __init__(self):
        super().__init__("navigate_through_poses_client")
        self._action_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )

    def send_goal(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses  # 将位姿列表传递给目标消息

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result:
            self.get_logger().info("Navigation completed successfully!")
        else:
            self.get_logger().error("Navigation failed!")

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


def main(args=None):
    rclpy.init(args=args)
    node = NavigateThroughPosesClient()

    single_pose_list = [
        
        # 0917 Morning 
        #INFO] [1726501751.963064218] [rviz2]: Setting estimate pose: Frame:map, Position(-0.0706844, 1.74007, 0), Orientation(0, 0, -0.763823, 0.645426) = Angle: -1.73843
        #[INFO] [1726501768.955983594] [rviz2]: Setting goal pose: Frame:map, Position(-0.22013, -5.46175, 0), Orientation(0, 0, -0.701131, 0.713033) = Angle: -1.55396
        #[1] [1726501791.035554691] [rviz2]: Setting goal pose: Frame:map, Position(-0.164133, -5.46306, 0), Orientation(0, 0, -0.695146, 0.718869) = Angle: -1.53724
        #[2] [1726501810.016164217] [rviz2]: Setting goal pose: Frame:map, Position(0.62992, -13.5636, 0), Orientation(0, 0, -0.400131, 0.916458) = Angle: -0.823319
        #[3] [1726501817.619129801] [rviz2]: Setting goal pose: Frame:map, Position(3.44506, -15.1877, 0), Orientation(0, 0, 0.00899489, 0.99996) = Angle: 0.01799
        #[4] [1726501852.149981555] [rviz2]: Setting goal pose: Frame:map, Position(6.51283, -14.7473, 0), Orientation(0, 0, 0.303469, 0.952841) = Angle: 0.616663
        #[5] [1726501857.408725958] [rviz2]: Setting goal pose: Frame:map, Position(6.97544, -11.5104, 0), Orientation(0, 0, 0.920148, 0.391571) = Angle: 2.33692
        #[6] [1726501863.419066992] [rviz2]: Setting goal pose: Frame:map, Position(4.19913, -7.80963, 0), Orientation(0, 0, 0.638938, 0.769258) = Angle: 1.38623
        #[7] [1726501870.578788556] [rviz2]: Setting goal pose: Frame:map, Position(5.84199, -3.89917, 0), Orientation(0, 0, 0.384651, 0.923062) = Angle: 0.789659
        #[8] [1726501876.075933388] [rviz2]: Setting goal pose: Frame:map, Position(7.66894, 0.888844, 0), Orientation(0, 0, 0.977853, 0.209292) = Angle: 2.71989
        #[9] Setting goal pose: Frame:map, Position(1.45356, 1.24954, 0), Orientation(0, 0, -0.858767, 0.512366) = Angle: -2.06572

        # 0917 Morning 
        # Point 1
        #{'x': -0.164133, 'y': -5.46306, 'z': 0.0, 'yaw': -1.53724},
        # Point 2
        #{'x': 0.62992, 'y': -13.5636, 'z': 0.0, 'yaw': -0.823319},
        # Point 3
        #{'x': 3.44506, 'y': -15.1877, 'z': 0.0, 'yaw': 0.01799},
        # Point 4
        #{'x': 6.51283, 'y': -14.7473, 'z': 0.0, 'yaw': 0.616663},
        # Point 5
        #{'x': 6.97544, 'y': -11.5104, 'z': 0.0, 'yaw': 2.33692},
        # Point 6
        #{'x': 4.19913, 'y': -7.80963, 'z': 0.0, 'yaw': 1.38623},
        # Point 7
        #{'x': 5.84199, 'y': -3.89917, 'z': 0.0, 'yaw': 0.789659},
        # Point 8
        #{'x': 7.66894, 'y': 0.888844, 'z': 0.0, 'yaw': 2.71989},
        # Point 9
        #{'x': 1.45356, 'y': 1.24954, 'z': 0.0, 'yaw': -2.06572}

        #"""
        #[INFO] [1726507100.576077552] [rviz2]: Trying to create a map of size 385 x 647 using 1 swatches
        #[1] [1726507126.989089964] [rviz2]: Setting goal pose: Frame:map, Position(0.0146346, -5.88931, 0), Orientation(0, 0, -0.713539, 0.700616) = Angle: -1.58907
        #[2] [1726507150.246942835] [rviz2]: Setting goal pose: Frame:map, Position(0.818586, -13.2416, 0), Orientation(0, 0, -0.330153, 0.943927) = Angle: -0.672932
        #[3] [1726507190.923888986] [rviz2]: Setting goal pose: Frame:map, Position(7.24757, -14.6056, 0), Orientation(0, 0, 0.537513, 0.843256) = Angle: 1.13497
        #[4] [1726507228.291385187] [rviz2]: Setting goal pose: Frame:map, Position(7.14134, -11.5419, 0), Orientation(0, 0, 0.924988, 0.379996) = Angle: 2.36201
        #[5] [1726507244.955982923] [rviz2]: Setting goal pose: Frame:map, Position(4.35308, -8.03755, 0), Orientation(0, 0, 0.692782, 0.721147) = Angle: 1.53068
        #[6] [1726507264.921227348] [rviz2]: Setting goal pose: Frame:map, Position(5.03375, -5.16274, 0), Orientation(0, 0, 0.440615, 0.897696) = Angle: 0.912567
        #[7] [1726507322.711020279] [rviz2]: Setting goal pose: Frame:map, Position(8.21546, 0.9646, 0), Orientation(0, 0, 0.947183, 0.320694) = Angle: 2.48867
        #[8] [1726507350.626487480] [rviz2]: Setting goal pose: Frame:map, Position(2.8724, 2.32594, 0), Orientation(0, 0, -0.905774, 0.42376) = Angle: -2.26641
        #"""

        # Point 1 直道中央开始
        {'x': 0.0146346, 'y': -5.88931,'z': 0.0, 'yaw': -1.58907},
        
        # Point 2 右下角弯 1号 弯心
        {'x': 0.379164, 'y': -13.9076, 'z': 0.0, 'yaw': -0.503171},
            # [rviz2]: Setting goal pose: Frame:map, Position(0.433538, -13.4188, 0), Orientation(0, 0, -0.420325, 0.907374) = Angle: -0.867606
            # Setting goal pose: Frame:map, Position(0.379164, -13.7076, 0), Orientation(0, 0, -0.24894, 0.968519) = Angle: -0.503171
        # Point 3 右上角弯 2号 进弯
        {'x': 7.34757, 'y': -14.6056,  'z': 0.0, 'yaw': 1.13497},
        # Point 4 右上角弯 2号 弯心
        {'x': 7.54134, 'y': -11.5419,  'z': 0.0, 'yaw': 2.36201},
        # Point 5 上方直线点
        #{'x': 4.15308, 'y': -7.53755,  'z': 0.0, 'yaw': 1.53068},
        {'x': 4.50308, 'y': -8.03755,  'z': 0.0, 'yaw': 1.53068},# speed
        # Point 6 上方弯  3号 左上方直道
        # {'x': 5.98572, 'y': -3.86257,  'z': 0.0, 'yaw': 0.969711},
        {'x': 5.88572, 'y': -3.86257,  'z': 0.0, 'yaw': 0.969711}, # speed
            # Setting goal pose: Frame:map, Position(5.88572, -3.86257, 0), Orientation(0, 0, 0.466081, 0.884742) = Angle: 0.969711
        # Point 7 左上角弯 4号 出弯带斜率
        {'x': 8.21546, 'y': 0.7646,  'z': 0.0, 'yaw': 2.48867},
        # Point 8 左下角弯 5号 直线出弯
        {'x': 3.98559, 'y': 2.62511,  'z': 0.0, 'yaw': -2.61212},
        {'x': 0.744836, 'y': -1.19677,  'z': 0.0, 'yaw': -1.77693}
            # 修改为两个
            #  Setting goal pose: Frame:map, Position(4.88559, 2.52511, 0), Orientation(0, 0, -0.965162, 0.261653) = Angle: -2.61212
        #  Setting goal pose: Frame:map, Position(0.844836, -1.19677, 0), Orientation(0, 0, -0.776105, 0.630603) = Angle: -1.77693

        



    ]

    poses = []

    # 使用循环重复添加这些目标点
    for _ in range(100):  # 重复5次
        for pose_dict in single_pose_list:
            pose = node.create_pose(
                x=pose_dict['x'],
                y=pose_dict['y'],
                z=pose_dict['z'],
                yaw=pose_dict['yaw']
            )
            poses.append(pose)

    node.send_goal(poses)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
