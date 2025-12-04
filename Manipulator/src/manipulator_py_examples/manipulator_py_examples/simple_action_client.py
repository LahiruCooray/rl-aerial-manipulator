import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from manipulator_msgs.action import Fibonacci


class SimpleActionClient(Node):
    def __init__(self):
        super().__init__('simple_action_client')
        self.action_client = ActionClient(self, Fibonacci, 'fibonacci')

        self.action_client.wait_for_server() # Waits until the "fibonacci" action server is available.

        self.goal = Fibonacci.Goal()
        self.goal.order = 10

        # Sends the goal to the action server asynchronously.
        # Returns a Future that will complete when the server replies to the goal request (accept/reject).
        self.future = self.action_client.send_goal_async(  
            self.goal,   
            feedback_callback=self.feedbackCallback)  #While the goal is executing, the server may publish feedback many times. Each time calls back to feedbackCallback.
        
        self.future.add_done_callback(self.responseCallback)  # Registers self.responseCallback to run once the above Future completes.

    '''This function is automatically invoked when the goal request future (returned by send_goal_async) finishes. 
    That future represents the server’s accept/reject response for your goal.'''
    def responseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted: # Checks if the goal was accepted by the server.
            self.get_logger().info('Goal rejected :(')  
            return  # Early return if the goal was rejected.

        self.get_logger().info('Goal accepted :)')

        self.future = goal_handle.get_result_async()    # After the goal is accepted, this asks asynchronously the server for the terminal outcome of this goal (one of: succeeded / aborted / canceled).
        # Returns a Future that will complete exactly once—when the goal finishes.
        # This is separate from feedback (which arrives via your feedback_callback while the goal is running).
        self.future.add_done_callback(self.resultCallback)  # Registers self.resultCallback to run once the above Future completes.

    def resultCallback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedbackCallback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.partial_sequence))


def main():
    rclpy.init()  # Initializes the ROS 2 Python client library. Must be called once before creating any node.
    action_client = SimpleActionClient()   # Creates an instance of the SimpleActionClient node.
    rclpy.spin(action_client)    # Spins the node, processing callbacks (like feedback and result) until shutdown.

if __name__ == '__main__':
    main()
    


# cd ~/manipulator_ws
# colcon build

# . install/setup.bash
# ros2 run manipulator_py_examples simple_action_client

# . install/setup.bash
# ros2 run manipulator_py_examples simple_action_client