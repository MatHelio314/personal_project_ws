import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


robot = 'smart_diffbot'
world = 'empty_world'


def generate_launch_description():
    
    
    ## Robot model
    xacro_file = os.path.join(get_package_share_directory(robot+'_description'), 'model', 'urdf', 'robot.urdf.xacro')


    ## Launch files


    ## Nodes

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' sim:=False']), value_type=str)
            }],
        emulate_tty=True
    )


    ## Launch description
    return LaunchDescription([

        # Launch

        # Nodes
        robot_state_publisher,
    ])
    
