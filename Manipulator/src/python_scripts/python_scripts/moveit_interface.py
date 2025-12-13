import rclpy
from rclpy.logging import get_logger
import numpy as np
import time
from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState


def move_arm():
    # MoveItPy will get configuration from parameters set by launch file
    manipulator = MoveItPy(node_name="moveit_interface_node")
    
    # Wait for controllers to be available
    get_logger("moveit_interface").info("Waiting for controllers to be available...")
    time.sleep(5)  # Give controllers time to start

    # Get planning components for arm and gripper
    arm = PlanningComponent("arm", manipulator)
    gripper = PlanningComponent("gripper", manipulator)

    # Create robot states for goals
    arm_state = RobotState(manipulator.get_robot_model())
    gripper_state = RobotState(manipulator.get_robot_model())
    
    # Set joint positions for goals
    arm_state.set_joint_group_positions("arm", [1.57, 0.0, 0.0])
    gripper_state.set_joint_group_positions("gripper", [0.5])  # Only joint_4, joint_5 mimics
    
    # Set start states
    arm.set_start_state_to_current_state()
    gripper.set_start_state_to_current_state()
    
    # Set goal states
    arm.set_goal_state(robot_state=arm_state)
    gripper.set_goal_state(robot_state=gripper_state)
    
    # Plan
    arm_plan = arm.plan()
    
    if arm_plan:
        get_logger("moveit_interface").info("Arm plan successful! Executing...")
        # Execute using the robot state from the plan
        manipulator.execute(arm_plan.trajectory, controllers=[])
    else:
        get_logger("moveit_interface").error("Arm planning failed!")
    
    # Plan gripper
    gripper_plan = gripper.plan()
    
    if gripper_plan:
        get_logger("moveit_interface").info("Gripper plan successful! Executing...")
        manipulator.execute(gripper_plan.trajectory, controllers=[])
    else:
        get_logger("moveit_interface").error("Gripper planning failed!")




def main():
    rclpy.init()
    move_arm()
    rclpy.shutdown()

if __name__ == '__main__':
    main()