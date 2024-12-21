# TurtleBot Maze Navigation

A ROS-based autonomous navigation system for TurtleBot that enables it to navigate through maze-like environments using laser scan data.

## Overview

This project implements an autonomous navigation algorithm for a TurtleBot robot equipped with a laser scanner (LIDAR). The robot can:
- Navigate through maze-like environments
- Detect and avoid obstacles using laser scan data
- Make intelligent turning decisions at barriers
- Use protective fields for safer navigation

## Technical Details

### Hardware Requirements
- TurtleBot robot
- Laser Scanner with:
  - 720 laser beams
  - 270-degree scan range
  - Field of view: Beam 0 to Beam 719

### Software Requirements
- ROS (Robot Operating System)
- Python 2.7 or higher
- Gazebo Simulator
- Required ROS packages:
  - geometry_msgs
  - sensor_msgs
  - nav_msgs
  - tf

## Project Structure

```
turtlebot_maze/
├── src/
│   ├── robot_control_class.py    # Base robot control class
│   └── maze_navigator.py         # Main navigation algorithm
├── launch/
│   └── maze_world.launch         # Gazebo world launch file
└── README.md
```

### Key Components

1. **RobotControl Class**
   - Base class providing fundamental robot control methods
   - Handles laser scan data processing
   - Manages robot movement and rotation
   - Implements odometry-based precise rotation

2. **Maze Navigator**
   - Implements the main navigation algorithm
   - Uses protective fields for obstacle detection
   - Handles turning decisions at barriers
   - Implements recursive navigation strategy

## Features

- **Protective Fields**: Uses multiple laser readings to create safety zones
- **Intelligent Turn Decision**: Evaluates both left and right clearance before turning
- **Optimized Sensor Reading**: Minimizes delay from laser scan readings (~1s per call)
- **Safety Measures**: Implements minimum distance thresholds and rotation limits

## Usage

1. Launch the Gazebo simulation:
```bash
roslaunch turtlebot_maze maze_world.launch
```
## Methods
The robot control class provides several key methods:

- `get_laser(pos)` : Get laser reading from specific position (0-719)
- `get_laser_full()`: Get all laser readings as a list
- `move_straight()`: Move robot forward
- `stop_robot()`: Stop all robot movement
- `move_straight_time(motion, speed, time)`: Controlled linear movement
- `turn(clockwise, speed, time)` : Basic turning functionality
- `rotate(degrees)` : Precise rotation using odometry

## Performance Considerations

- Laser readings take approximately 1 second per call
- Publishing messages has a delay of ~1 second
- Critical timing for obstacle detection and stopping
- Turn decisions require multiple sensor readings

## Future Improvements

- Implement caching for laser readings
- Add dynamic speed control based on proximity
- Enhance path finding algorithm
- Optimize sensor reading frequency
- Implement smoother turning behavior