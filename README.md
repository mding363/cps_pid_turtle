# Turtle Controller - ROS 2 Humble Package

A  ROS 2 package demonstrating Twist, Odometry and Imu message publishing, and subscribing using the turtlesim simulator.

## Overview

This package demonstrates:

- **Topic Publishers** - Sending Odometry/Imu type message
- **Controller** - Sending velocity command to control turtle movement and
                   Receiving turtle pose information to support turtle movement
- **Launch Files** - Starting multiple nodes simultaneously

You should see output similar to: `ros2 cli version: 0.18.x`

## Package Structure

```
cps_pid_turtle/
├── launch/
│   ├── follow_shape_launch.py          # Basic launch file
├── cps_pid_turtle/
│   ├── __init__.py
│   ├── controller.py                    # Publishes velocity commands
│   |__ sensor_publisher.py              # Publishes Odometry/Imu messages
├── package.xml                          # Package metadata
├── setup.py                             # Python package setup
├── setup.cfg                            # Package configuration
└── README.md                            # This file
```

## Quick Start

### 1. Create Workspace and Clone Package

```bash
# Create workspace
mkdir -p ~/turtle_ws/src
cd ~/turtle_ws/src

# Clone or copy this package into src/
# (If you created it manually, skip this step)

cd ~/turtle_ws
```

### 2. Install Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Package

```bash
cd ~/turtle_ws
colcon build
source install/setup.bash
```

### 4. Launch the Complete System

```bash
ros2 launch cps_pid_turtle follow_shape_launch.py
```

You should see:
- Turtlesim window opens
- Turtle moves in a triangle, velocity etc is printed to terminal

## Nodes Description

### 0. Run the turlesim simulation
```bash
ros2 run turtlesim turtlesim_node
```

### 1. Turtle Controller (`controller`)

Publishes `Twist` messages to `/turtle1/cmd_vel` to make the turtle move in a triangle pattern.

**Run standalone:**
```bash
ros2 run cps_pid_turtle controller
```

**Topics Published:**
- `/turtle1/cmd_vel` (Twist) - Velocity commands

**Topics Subscribed:**
- `/turtle1/pose` - Get turtle pose

### 2. Sensor Publisher (`sensor_publisher`)

Publishes `Odometry` and `Imu` message to `/nav_msgs/odometry` and `/sensor_msgs/imu`

## Development

### Building After Changes

After modifying any Python files:

```bash
cd ~/turtle_ws
colcon build --packages-select turtle_controller
source install/setup.bash
```

### Adding New Nodes

1. Create your Python script in `turtle_controller/`
2. Make it executable: `chmod +x your_script.py`
3. Add entry point in `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'your_node = turtle_controller.your_script:main',
       ],
   },
   ```
4. Rebuild the package


##  Useful Commands

### Inspect Running System

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# See topic info
ros2 topic info /turtle1/cmd_vel

# Echo topic messages
ros2 topic echo /turtle1/pose

# List services
ros2 service list

# See service type
ros2 service type /rotate_turtle

# View computational graph
rqt_graph
```
