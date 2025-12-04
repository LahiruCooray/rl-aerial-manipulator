import rclpy
from rclpy.node import Node
from manipulator_msgs.srv import AddTwoInts



class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        
        self.service_ = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server is ready to add two integers.')

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('New Message Received, a: %d b: %d' % (request.a, request.b))
        response.sum = request.a + request.b
        self.get_logger().info('Sending back response: %d' % response.sum)
        return response
    

def main():
    rclpy.init()
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#1  Colcon build
#2  Source the install/setup.bash,  ros2 run manipulator_py_examples simple_service_server
#3  ros2 service list, . install/setup.bash, ros2 service type /add_two_ints, ros2 service call /add_two_ints manipulator_msgs/srv/AddTwoInts "{a: 5, b: 3}"