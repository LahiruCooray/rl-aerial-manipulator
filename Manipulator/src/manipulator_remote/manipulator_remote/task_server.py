#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from manipulator_msgs.action import ManipulatorTask
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class TaskServer(Node):
    def __init__(self):
        super().__init__('task_server')
        self.get_logger().info('Starting the Server...')
        self.action_server = ActionServer(
            self,
            ManipulatorTask,
            'task_server',
            self.goalCallback,
        )
    
        self.manipulator = MoveItPy(node_name="moveit_py")
        self.manipulator_arm = self.manipulator.get_planning_component("arm")
        self.manipulator_gripper = self.manipulator.get_planning_component("gripper")

        self.get_logger().info('Action Server has been started.')

    def goalCallback(self, goal_handle):
        self.get_logger().info('Received goal request with task number %d' % goal_handle.request.task_number)

        arm_state = RobotState(self.manipulator.get_robot_model())
        gripper_state = RobotState(self.manipulator.get_robot_model())

        arm_joint_goal =[]
        gripper_joint_goal = []

        if goal_handle.request.task_number == 0:
            arm_joint_goal = np.array([0.0, 0.0, 0.0])
            gripper_joint_goal = np.array([1.0, -1.0]) 
        elif goal_handle.request.task_number == 1:
            arm_joint_goal = np.array([-1.14, 1.0, 0.7])
            gripper_joint_goal = np.array([0.0, 0.0])
        elif goal_handle.request.task_number == 2:
            arm_joint_goal = np.array([1.57, 0.5, -0.3])
            gripper_joint_goal = np.array([0.5, -0.5])
        else:
            self.get_logger().info('Invalid task number received.')
            return
        
        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)

        self.manipulator_arm.set_start_state_to_current_state()
        self.manipulator_gripper.set_start_state_to_current_state()

        self.manipulator_arm.set_goal_state(robot_state=arm_state)
        self.manipulator_gripper.set_goal_state(robot_state=gripper_state)

        arm_plan_result = self.manipulator_arm.plan()
        gripper_plan_result = self.manipulator_gripper.plan()

        if arm_plan_result and gripper_plan_result:
            self.manipulator.execute(arm_plan_result.trajectory, controllers=[])
            self.manipulator.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().info('Planning failed for the given task number.')
        
        goal_handle.succeed()
        result = ManipulatorTask.Result()
        result.success = True
        return result

    

def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)
    # task_server.destroy_node()
    # rclpy.shutdown()

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