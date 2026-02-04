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
