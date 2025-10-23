import sys
import rclpy
from rclpy.node import Node
from my_own_service.srv import AddTwoInt


def send_request(client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future


def main(args=None):
    global node
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')

    client = node.create_client(AddTwoInt, 'add_two_ints')

    while not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('Service not available, waiting again...')

    request = AddTwoInt.Request()
    request.a = int(sys.argv[1])
    request.b = int(sys.argv[2])

    response = send_request(client, request)

    if response.result() is not None:
        node.get_logger().info(f"Result: {response.result().sum}")
    else:
        node.get_logger().error('Service call failed!')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

