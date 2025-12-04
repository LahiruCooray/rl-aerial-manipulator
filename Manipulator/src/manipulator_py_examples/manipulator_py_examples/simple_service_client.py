import rclpy
from rclpy.node import Node
from manipulator_msgs.srv import AddTwoInts
import sys


class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__('simple_service_client')
        
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request_ = AddTwoInts.Request()
        self.request_.a = a
        self.request_.b = b
        self.future_ = self.client_.call_async(self.request_)
        self.future_.add_done_callback(self.response_callback)

    def response_callback(self, future):
        self.get_logger().info('Response received: %d' % future.result().sum)


def main():
    rclpy.init()
    
    if len(sys.argv) != 3:
        print("Wrong number of arguments. Usage: simple_service_client <int a> <int b>")
        return -1

    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



#1  Colcon build
#2  Source the install/setup.bash,  ros2 run manipulator_py_examples simple_service_server
#3  . install/setup.bash, ros2 run manipulator_py_examples simple_service_client 5 3
    
    
