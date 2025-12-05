import os   #Used for filesystem operations (e.g., joining paths, checking file existence).
from ament_index_python.packages import get_package_share_directory #Returns the absolute path to the share directory of a given ROS 2 package.

from launch import LaunchDescription 
#Imports the LaunchDescription class, which defines the structure of the launch file.
#Every ros2 launch file must return a LaunchDescription object containing nodes, arguments and other launch actions.
"""
def generate_launch_description():
    return LaunchDescription([nodes_and_actions_here])
"""

from launch.actions import DeclareLaunchArgument 
#Imports the DeclareLaunchArgument action, which allows defining command-line arguments for the launch file.
#Lets users override parameters (e.g., URDF file paths, simulation settings).
"""
model_arg = DeclareLaunchArgument(
    name="model",
    default_value="path/to/default.urdf",
    description="Description of the argument"
)
""" #Accessed later via : LaunchConfiguration('model') 

from launch.substitutions import LaunchConfiguration, Command
#LaunchConfiguration('model') : Retrieves the value of a launch argument (DeclareLaunchArgument).
#Command : Executes a shell command and retrieves its output. Executes shell commands (e.g., xacro to process URDclearF/Xacro files).
#Used to get the absolute path of the URDF file.
"""
urdf_path = Command(['xacro', LaunchConfiguration('model')])
"""

from launch_ros.actions import Node 
#Imports the Node action, which allows launching ROS 2 nodes.
#Node is a class that represents a single ROS 2 node to be launched.
#It takes care of setting up the node's parameters, remappings, and other configurations.
"""
Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": robot_description}]
)
"""

from launch_ros.parameter_descriptions import ParameterValue
#Imports ParameterValue, which helps define ROS parameters with explicit types.
#Ensures parameters are passed with correct data types (e.g., str, bool, float).
"""
param_value = ParameterValue(value, type=ParameterType.STRING)
"""

def generate_launch_description():
    #This function generates the launch description for the ROS 2 launch file.
    #It defines the nodes, parameters, and other configurations needed to run the simulation.
    #The function is called when the launch file is executed.

    manipulator_description_dir = get_package_share_directory("manipulator_description")
    #Get the absolute path to the share directory of the manipulator_description package.
    #This path is used to locate the URDF file and other resources needed for the simulation.

    urdf_file_dir = os.path.join(manipulator_description_dir, "urdf", "manipulator.urdf.xacro")
    #Join the package directory with the relative path to the URDF file.

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=urdf_file_dir,
        description='Absolute path to robot urdf file'
    )    
    #Declare a launch argument named 'model' with a default value pointing to the URDF file.

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    ) 
    # Convert the .xacro file into a URDF XML string at launch time.
    #Uses the xacro command to process the URDF/XACRO file.
    #Gets the file path from the "model" launch configuration, Specifies the value type as string.

    robot_state_publisher_node = Node(
        package="robot_state_publisher",   
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )  #"From the robot_state_publisher package, run the program (node) called robot_state_publisher."
    #Pass the robot_description parameter to the node, which contains the processed URDF/XACRO file.
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    #ROS 2 tool that provides a Graphical User Interface (GUI) to manually control or simulate robot joint positions

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(manipulator_description_dir, "rviz", "display.rviz")],
    )
    #Launches RViz2 (ROS visualization tool), Loads a predefined configuration from "display.rviz"
    #Shows output on screen (not just logging)

    '''
    ✅ robot_state_publisher_node : publishes the TF tree based on the robot description and joint states
    ✅ joint_state_publisher_gui_node : provides GUI sliders to simulate joint movement
    ✅ rviz_node : opens RViz2 for 3D visualization
    '''

    #It returns a LaunchDescription object that contains all the necessary components to launch the simulation.
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
