import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


robot = 'smart_diffbot'


def generate_launch_description():

    ## Arguments
    sim_arg = DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Run robot in simulation (sim:=true) or use real hardware (sim:=false)'
        )
    sim = LaunchConfiguration('sim')

    ## Launch simulation
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'simulation.launch.py')]),
        condition=IfCondition(sim),
    )

    ## or launch actual hardware
    launch_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'hardware.launch.py')]),
        condition=UnlessCondition(sim),
    ) 

    ## Launch control
    launch_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_control'), 'launch', 'control.launch.py')]),
    )

    balance_controller = Node(
        package='smart_diffbot_bringup',  
        executable='balance_controller',  
        output='screen'
    )

    pid_controller = Node(
        package='smart_diffbot_bringup',  
        executable='pid_controller',  
        output='screen'
    )

 
    ## Launch description
    return LaunchDescription([

        # Arguments
        sim_arg,

        # Launch
        # balance_controller,
        launch_simulation,
        launch_hardware,
        launch_control,
        # pid_controller,
    ])
    
