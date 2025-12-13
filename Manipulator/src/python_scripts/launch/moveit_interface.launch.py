import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("manipulator", package_name="manipulator_moveit")
        .robot_description( file_path=os.path.join(
            get_package_share_directory("manipulator_description"), 
            "urdf", 
            "manipulator.urdf.xacro"
            ) 
        )
        .robot_description_semantic( file_path="config/manipulator.srdf" )
        .trajectory_execution( file_path="config/moveit_controllers.yaml" )
        .moveit_cpp( file_path="config/planning_python_api.yaml" )
        .to_moveit_configs()
    )

    moveit_interface = Node(
        package="python_scripts",
        executable="moveit_interface",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True}]
    )

    return LaunchDescription([
        moveit_interface
    ])