from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def get_robot_description(context, *args, **kwargs):
    dof = LaunchConfiguration('dof').perform(context)
    pkg_share = FindPackageShare('so_100_arm').find('so_100_arm')
    urdf_path = os.path.join(pkg_share, 'urdf', f'so_100_arm_{dof}dof.urdf')
    controller_path = os.path.join(pkg_share, 'config', f'controllers_{dof}dof.yaml')

    with open(urdf_path, 'r') as file:
        urdf_content = file.read()
        return {
            'robot_description': ParameterValue(urdf_content, value_type=str),
            'controller_path': controller_path
        }

def generate_launch_description():
    dof_arg = DeclareLaunchArgument(
        'dof',
        default_value='5',
        description='DOF configuration - either 5 or 7'
    )

    def launch_setup(context, *args, **kwargs):
        descriptions = get_robot_description(context)

        # Launch Gazebo Classic
        gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        )

        # Start robot_state_publisher
        state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': descriptions['robot_description']}]
        )

        # Spawn robot into Gazebo using robot_description
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=[
                '-entity', 'so_100_arm',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0'
            ],
            output='screen'
        )

        # Controllers
        joint_state_broadcaster_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'joint_state_broadcaster'],
            output='screen'
        )

        joint_trajectory_controller_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'joint_trajectory_controller'],
            output='screen'
        )

        gripper_controller_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'gripper_controller'],
            output='screen'
        )

        return [
            gazebo,
            state_pub,
            spawn_robot,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_broadcaster_spawner]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[joint_trajectory_controller_spawner]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_trajectory_controller_spawner,
                    on_exit=[gripper_controller_spawner]
                )
            )
        ]

    return LaunchDescription([
        dof_arg,
        OpaqueFunction(function=launch_setup)
    ])
