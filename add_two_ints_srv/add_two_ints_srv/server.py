import rclpy
from rclpy.node import Node
from my_own_service.srv import AddTwoInt


class AddTwoIntServer(Node):
    def __init__(self):
        super().__init__('add_two_int_server')
        self.srv = self.create_service(AddTwoInt, 'add_two_ints', self.add_two_int_callback)
        self.get_logger().info('Service Server Ready: [add_two_ints]')

    def add_two_int_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"Incoming request\na: {request.a}, b: {request.b}, sum: {response.sum}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

