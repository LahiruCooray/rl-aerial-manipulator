#!/usr/bin/env python3

"""
Launch file for the complete aerial manipulator system.
Starts:
  1. Gazebo with hexacopter + manipulator
  2. PX4 SITL
  3. ROS-Gazebo bridge
  4. Manipulator controllers (ros2_control)
  5. MoveIt2 for the arm
  6. Static TF between hexacopter and manipulator
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    # Paths
    workspace_root = '/home/lahiru/newlocalrepo/rl-aerial-manipulator'
    world_file = os.path.join(workspace_root, 'aerial_manipulator_bringup', 'worlds', 'hexa_with_arm.sdf')
    
    # Set Gazebo resource path so it can find model meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(workspace_root, 'hexacopter_description') + ':' + 
              os.path.join(workspace_root, 'Manipulator/src/manipulator_description/sdf')
    )
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. Start Gazebo with the combined world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        shell=False
    )
    
    # 2. Static TF: base_link (hexacopter) -> base_plate (manipulator)
    # This must match the physical mounting offset in the SDF world file
    # Since both are at z=1.0, the offset is (0, 0, 0)
    static_tf_hexa_to_arm = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hexa_to_arm_tf',
        arguments=[
            '0', '0', '0',       # x, y, z offset (touching, no gap)
            '0', '0', '0',       # roll, pitch, yaw
            'base_link',         # parent frame (hexacopter body)
            'base_plate'         # child frame (manipulator base)
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 3. Robot state publisher for manipulator
    # TODO: Update this path to your actual URDF/xacro file
    manipulator_urdf = os.path.join(
        workspace_root, 
        'Manipulator/src/manipulator_description/urdf/manipulator.urdf.xacro'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(manipulator_urdf).read(),
            'use_sim_time': use_sim_time
        }]
    )
    
    # 4. ROS-Gazebo bridge for topics
    # Bridge important topics (adjust as needed for your setup)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Add more topic bridges as needed:
            # '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )
    
    # 5. Start manipulator controllers
    # TODO: Include your manipulator's controller manager and spawn controllers here
    # Example:
    # controller_manager = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(workspace_root, 'Manipulator/launch/controllers.launch.py')
    #     ])
    # )
    
    # 6. Start MoveIt2 for the arm
    # TODO: Include your MoveIt launch file here
    # Example:
    # moveit = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(workspace_root, 'Manipulator/launch/moveit.launch.py')
    #     ])
    # )
    
    # 7. PX4 SITL (if using PX4)
    # TODO: Add PX4 launch if you're using it
    # Example:
    # px4_sitl = ExecuteProcess(
    #     cmd=['make', 'px4_sitl', 'gz_custom_hexa'],
    #     cwd='/path/to/PX4-Autopilot',
    #     output='screen'
    # )
    
    return LaunchDescription([
        # Set environment variable for Gazebo to find models
        gz_resource_path,
        
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Nodes and processes
        gazebo,
        static_tf_hexa_to_arm,
        robot_state_publisher,
        ros_gz_bridge,
        
        # TODO: Uncomment and configure these when ready:
        # controller_manager,
        # moveit,
        # px4_sitl,
    ])
