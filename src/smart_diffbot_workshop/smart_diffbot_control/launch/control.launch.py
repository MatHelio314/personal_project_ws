import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument,TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():


    # Spawn joint state broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )


    ## Arguments
    sim_arg = DeclareLaunchArgument(name='sim', default_value='true',choices=['true', 'false'], description='Set to true to switch from hardware to simulation in the loop')

    ## Parameters
    controller_params = os.path.join(get_package_share_directory('smart_diffbot_control'), 'config', 'controller_params.yaml')

    ## Controller manager (only on real robot, Gazebo start one by default)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params],
        remappings=[('~/robot_description', '/robot_description'),],
        condition=UnlessCondition(LaunchConfiguration('sim')),
        emulate_tty=True,
    )

    # Spawn diff drive controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller","--controller-manager", "/controller_manager"],
    )
    delayed_delayed_diff_drive_spawner = TimerAction(period=3.0, actions=[diff_drive_spawner])

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[delayed_delayed_diff_drive_spawner]
        )
    )  


    ## Launch description
    return LaunchDescription([
        sim_arg,
        ros2_control_node,
        joint_broad_spawner,
        delayed_diff_drive_spawner,
    ])