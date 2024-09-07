import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher_node")

        # 订阅路径话题
        self.create_subscription(Path, "/planning/global_path", self.path_callback, 1)

        # 创建 FollowPath 的 Action Client
        self.nav_through_poses_client = ActionClient(self, FollowPath, "follow_path")

        self.get_logger().info("---Path publisher node initialized---")

    def path_callback(self, msg: Path):
        """接收路径并发送给 `FollowPath` action server"""
        self.get_logger().info(f"Spline Path Received - length: {len(msg.poses)}")
        self.send_path(msg)

    def send_path(self, path: Path):
        """发送 `FollowPath` 的 action 请求"""
        # 等待 action server 可用
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                "'FollowPath' action server not available, waiting..."
            )

        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        # 发送控制器ID到请求中
        goal_msg.controller_id = "RegulatedPurePursuit"

        # 发送路径
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted, waiting for result")

        # 获取结果的异步回调
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal completed with result: {result}")


def main(args=None):
    rclpy.init(args=args)

    node = PathPublisher()

    try:
        rclpy.spin(node)  # 让节点保持运行，直到按下 Ctrl+C 退出
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
