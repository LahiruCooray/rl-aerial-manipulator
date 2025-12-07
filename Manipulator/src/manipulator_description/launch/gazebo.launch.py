import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    manipulator_description_dir = get_package_share_directory("manipulator_description")

    urdf_file_dir = os.path.join(manipulator_description_dir, "urdf", "manipulator.urdf.xacro")

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=urdf_file_dir,
        description='Absolute path to robot URDF file'
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            str(Path(manipulator_description_dir).parent.resolve())
        ]  
    )

    is_ignition = 'false'
    physics_engine = "--physics-engine gz-physics-bullet-featherstone-plugin"

    robot_description = ParameterValue(
        Command([
            "xacro ", 
            LaunchConfiguration('model'), 
            " is_ignition:=",
            is_ignition
            ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]     # Use simulation time used to sync with Gazebo
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 
                'launch'
            ), '/gz_sim.launch.py']
        ),
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf ", physics_engine])
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'manipulator']
    )

    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
