# SO-100 Robot Arm ROS2 Package

This package provides ROS2 support for the SO-100 robot arm, available in both 5-DOF and 7-DOF configurations. It is based on the open-source 3D printable [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) project by The Robot Studio. This implementation includes URDF models, Gazebo simulation support, and MoveIt2 integration.

The original ROS1 implementation can be found at: https://github.com/TheRobotStudio/SO-ARM100

## Features

- Robot arm URDF models
  - 5-DOF configuration
  - 7-DOF configuration
- Gazebo Harmonic simulation support
- ROS2 Control integration
- Joint trajectory controller configuration
- MoveIt2 motion planning capabilities (In Progress)
  - Basic configuration generated
  - Integration with Gazebo pending
  - Motion planning testing pending

## Prerequisites

- ROS2 Humble
- Gazebo Garden
- MoveIt2
- ros2_control
- gz_ros2_control

## Installation

### Create a ROS2 workspace (if you don't have one)

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

### Clone the repository

git clone <repository_url>

### Install dependencies

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

### Build the package

colcon build --packages-select so_100_arm
source install/setup.bash

## Usage

### Launch the robot in Gazebo

```bash
ros2 launch so_100_arm gz.launch.py dof:5
```

### Launch the robot in RVIZ

```bash
ros2 launch so_100_arm rviz.launch.py
```

### Launch MoveIt2 Demo

```bash
ros2 launch so_100_arm demo.launch.py
```

### Test Joint Movement

#### Send a test position command for 7dof arm

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["Shoulder_Pitch", "Shoulder_Yaw", "Humeral_Rotation", "Elbow", "Wrist_Roll", "Wrist_Yaw", "Wrist_Pitch"], points: [{positions: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 1, nanosec: 0}}]}'
```

#### Send a test position command for 5dof arm

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["Shoulder_Rotation", "Shoulder_Pitch", "Elbow", "Wrist_Roll", "Wrist_Pitch"], points: [{positions: [1.0, 1.0, 1.0, 1.0, 1.0], velocities: [], accelerations: [], effort: [], time_from_start: {sec: 1, nanosec: 0}}]}'
```

## Package Structure

so_100_arm/  
├── CMakeLists.txt                      # Build system configuration  
├── config/  
│   ├── controllers_5dof.yaml           # 5DOF joint controller configuration  
│   ├── controllers_7dof.yaml           # 7DOF joint controller configuration  
│   ├── initial_positions.yaml          # Default joint positions  
│   ├── joint_limits.yaml               # Joint velocity and position limits  
│   ├── kinematics.yaml                 # MoveIt kinematics configuration  
│   ├── moveit_controllers.yaml         # MoveIt controller settings  
│   ├── moveit.rviz                     # RViz configuration for MoveIt  
│   ├── pilz_cartesian_limits.yaml      # Cartesian planning limits  
│   ├── ros2_controllers.yaml           # ROS2 controller settings  
│   ├── sensors_3d.yaml                 # Sensor configuration  
│   ├── so_100_arm.ros2_control.xacro   # ROS2 Control macro  
│   ├── so_100_arm.srdf                 # Semantic robot description  
│   ├── so_100_arm.urdf.xacro          # Main robot description macro  
│   └── urdf.rviz                       # RViz configuration for URDF  
├── launch/  
│   ├── demo.launch.py                  # MoveIt demo with RViz  
│   ├── gz.launch.py                    # Gazebo simulation launch  
│   ├── move_group.launch.py            # MoveIt move_group launch  
│   ├── moveit_rviz.launch.py           # RViz with MoveIt plugin  
│   ├── rsp.launch.py                   # Robot state publisher  
│   ├── rviz.launch.py                  # Basic RViz visualization  
│   ├── setup_assistant.launch.py       # MoveIt Setup Assistant  
│   ├── spawn_controllers.launch.py      # Controller spawning  
│   ├── static_virtual_joint_tfs.launch.py  
│   └── warehouse_db.launch.py          # MoveIt warehouse database  
├── LICENSE  
├── models/  
│   ├── so_100_arm_5dof/               # 5DOF robot assets  
│   │   ├── meshes/                    # STL files for visualization  
│   │   └── model.config               # Model metadata  
│   └── so_100_arm_7dof/               # 7DOF robot assets  
│       ├── meshes/                    # STL files for visualization  
│       └── model.config               # Model metadata  
├── package.xml                         # Package metadata and dependencies  
├── README.md                           # This documentation  
└── urdf/  
    ├── so_100_arm_5dof.csv            # Joint configuration data  
    ├── so_100_arm_5dof.urdf           # 5DOF robot description  
    ├── so_100_arm_7dof.csv            # Joint configuration data  
    └── so_100_arm_7dof.urdf           # 7DOF robot description  

## Joint Configuration

### 7-DOF Configuration
1. Shoulder Pitch     (-3.14 to 3.14 rad)
2. Shoulder Yaw      (-1.0 to 0.0 rad)
3. Humeral Rotation  (-1.8 to 1.8 rad)
4. Elbow            (-3.0 to 0.0 rad)
5. Wrist Roll       (-1.8 to 1.8 rad)
6. Wrist Yaw        (-0.9 to 0.5 rad)
7. Wrist Pitch      (-1.8 to 1.8 rad)

### 5-DOF Configuration
1. Shoulder Rotation (-3.14 to 3.14 rad)
2. Shoulder Pitch    (-3.14 to 3.14 rad)
3. Elbow            (-3.14 to 3.14 rad)
4. Wrist Pitch      (-3.14 to 3.14 rad)
5. Wrist Roll       (-3.14 to 3.14 rad)

Note: The 5-DOF configuration uses continuous rotation joints with full range of motion (±π radians).

## Known Issues

- The MoveIt2 configuration is still in development
- Some joint limits may need fine-tuning
- Collision checking needs optimization

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the Apache License - see the LICENSE file for details

## Authors

Bruk G.

## Acknowledgments

- Based on ROS2 Humble
- Uses MoveIt2 for motion planning
- Gazebo Harmonic for simulation
