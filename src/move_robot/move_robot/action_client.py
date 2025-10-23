import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from reach_wall_action.action import ReachWall  # ⚠️ use your actual action package name
import sys

class ReachWallClient(Node):

    def __init__(self):
        super().__init__('reach_wall_action_client')
        self._action_client = ActionClient(self, ReachWall, 'reach_wall')

    def send_goal(self, target_distance):
        goal_msg = ReachWall.Goal()
        goal_msg.target_distance = target_distance

        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal request: target_distance = {target_distance}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected 😢')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted ✅')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action finished. Reached: {result.reached}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Remaining distance: {feedback.remaining_distance:.3f}')


def main(args=None):
    rclpy.init(args=args)
    action_client = ReachWallClient()

    if len(sys.argv) < 2:
        print("Usage: ros2 run move_robot action_client <target_distance>")
        rclpy.shutdown()
        return

    target_distance = float(sys.argv[1])
    action_client.send_goal(target_distance)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
