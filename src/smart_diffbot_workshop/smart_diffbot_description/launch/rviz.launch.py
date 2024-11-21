import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

robot = 'smart_diffbot'

def generate_launch_description():
    
    ## Robot model
    xacro_file = os.path.join(get_package_share_directory(robot+'_description'), 'model', 'urdf', 'robot.urdf.xacro')


    ## Nodes

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name = "robot_state_publisher",
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name = "joint_state_publisher_gui"
    )

    # Rviz
    rviz_config = os.path.join(get_package_share_directory(robot+'_description'), 'launch', 'rviz', 'view_model.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    ## Launch description
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
    
