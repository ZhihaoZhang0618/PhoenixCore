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
        {'x': 1.3, 'y': -5.8, 'z': 0.0, 'yaw': -3.4},  # 第三个目标点，yaw=180度
        {'x': 10.89, 'y': -1.56, 'z': 0.0, 'yaw':-3.07},  # 第一个目标点
        {'x': -2.07, 'y': -1.15, 'z': 0.0, 'yaw': -3.14}  # 第二个目标点，yaw=90度
    ]

    poses = []

    # 使用循环重复添加这些目标点
    for _ in range(5):  # 重复5次
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
