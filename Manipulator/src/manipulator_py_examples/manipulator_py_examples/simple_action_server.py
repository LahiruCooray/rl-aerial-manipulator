import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from manipulator_msgs.action import Fibonacci
import time

class SimpleActionServer(Node):
    def __init__(self):
        super().__init__('simple_action_server')
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.goalCallback,
        )
        self.get_logger().info('Action Server has been started.')

    def goalCallback(self, goal_handle):
        self.get_logger().info('Received goal request with order %d' % goal_handle.request.order)

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1])
            self.get_logger().info('Publishing feedback: %s' % str(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)  

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info('Goal succeeded with result: %s' % str(result.sequence))
        return result
    

def main():
    rclpy.init()
    simple_action_server = SimpleActionServer()
    rclpy.spin(simple_action_server)
    simple_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# cd ~/manipulator_ws
# colcon build

# . install/setup.bash
# ros2 run manipulator_py_examples simple_action_server

# ros2 action list
# . install/setup.bash
# ros2 action info /fibonacci -t
# ros2 action send_goal /fibonacci manipulator_msgs/action/Fibonacci "{order: 10}" -f