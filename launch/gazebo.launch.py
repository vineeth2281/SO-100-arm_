from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():
    pkg_path = get_package_share_path('so_100_arm')
    urdf_path = os.path.join(pkg_path, 'urdf', 'so_100_arm.urdf')
    
    # Set the package path for Gazebo
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += f":{pkg_path}"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = pkg_path

    # Create robot state publisher node with URDF
    with open(urdf_path, 'r') as file:
        urdf_content = file.read()
        # Convert package:// to model:// for Gazebo
        gazebo_urdf_content = urdf_content.replace(
            'package://so_100_arm/models/so_100_arm/meshes',
            'model://so_100_arm/meshes'
        )
        
        # Keep original content for robot_state_publisher
        robot_description = ParameterValue(
            urdf_content,
            value_type=str
        )
        
        # Use modified content for Gazebo
        gazebo_description = ParameterValue(
            gazebo_urdf_content,
            value_type=str
        )

    return LaunchDescription([
        # Start robot_state_publisher with original URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen',
            additional_env={'GZ_SIM_RESOURCE_PATH': os.environ['GZ_SIM_RESOURCE_PATH']}
        ),

        # Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            parameters=[{
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
        ),

        # Spawn robot model using Gazebo-modified URDF
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_model',
            arguments=[
                '-string', gazebo_description.value,
                '-name', 'so_100_arm',
                '-allow_renaming', 'true',
                '-x', '0',
                '-y', '0',
                '-z', '0'
            ],
            output='screen'
        )
    ])