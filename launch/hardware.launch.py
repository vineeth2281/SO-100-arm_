from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    # Add launch argument
    test_zero_pose_arg = DeclareLaunchArgument(
        'test_zero_pose',
        default_value='false',
        description='Test zero pose after startup'
    )

    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution(
                    [FindPackageShare('so_100_arm'), 'urdf', 'so_100_arm_5dof_hardware.urdf']
                )
            ]
        ),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [FindPackageShare('so_100_arm'), 'config', 'controllers.yaml']
            ),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after joint_state_broadcaster
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["so_100_arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["so_100_arm_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Delay loading and starting robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('so_100_arm'), 'config', 'urdf.rviz'])]
    )

    # Add zero pose test node
    zero_pose_node = Node(
        package='so_arm_100_hardware',
        executable='zero_pose.py',
        name='zero_pose_test',
        condition=IfCondition(LaunchConfiguration('test_zero_pose'))
    )

    nodes = [
        robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        gripper_controller_spawner,
        rviz_node,
        zero_pose_node
    ]

    return LaunchDescription([test_zero_pose_arg] + nodes) 