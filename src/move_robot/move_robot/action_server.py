import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from reach_wall_action.action import ReachWall  # replace with your package name
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

distance_to_the_wall = float('inf')

def lidar_callback(msg):
    global distance_to_the_wall
    # Take minimum distance in front sector
    front_ranges = msg.ranges[0:10] + msg.ranges[-10:]
    distance_to_the_wall = min(front_ranges)
    

def execute_callback(goal_handle):
    global distance_to_the_wall, vel_publisher, node

    node.get_logger().info(f"Received goal request: target_distance = {goal_handle.request.target_distance}")
    goal_distance = goal_handle.request.target_distance

    feedback_msg = ReachWall.Feedback()
    result_msg = ReachWall.Result()

    robot_vel = Twist()

    while distance_to_the_wall > goal_distance:
        rclpy.spin_once(node)
        robot_vel.linear.x = 0.1
        vel_publisher.publish(robot_vel)

        feedback_msg.remaining_distance = float(distance_to_the_wall)
        goal_handle.publish_feedback(feedback_msg)

        time.sleep(0.1)

    # Stop the robot
    robot_vel.linear.x = 0.0
    vel_publisher.publish(robot_vel)

    result_msg.reached = True
    goal_handle.succeed()
    node.get_logger().info("Goal reached. Stopping robot.")
    return result_msg


def main(args=None):
    global node, vel_publisher

    rclpy.init(args=args)
    node = rclpy.create_node('reach_wall_server')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    action_server = ActionServer(node, ReachWall, 'reach_wall', execute_callback)
    vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    node.create_subscription(LaserScan, '/scan', lidar_callback, 10)

    node.get_logger().info("ReachWall Action Server has started.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
