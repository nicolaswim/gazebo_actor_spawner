# gazebo_actor_relay
> Bridging Gazebo Classic actors into RViz2 with live markers and TF updates.

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue) ![License: MIT](https://img.shields.io/badge/License-MIT-green)

## Overview
Gazebo Classic can spawn animated humans, but RViz2 has no native way to render those actors or follow their transforms. `gazebo_actor_relay` closes that gap by listening to `/model_states`, filtering out `human_*` actors, and publishing both visualization markers and TF transforms so operators can see every actor inside RViz just like any other robot.

## Key Features
- **Dynamic Visuals** – Every actor is rendered as an easily visible green cylinder, making them stand out against the rest of the scene.
- **Velocity-Based Scaling** – Heights change with speed via `sqrt(vx^2 + vy^2)`; stationary actors remain 1.0 m tall while fast ones tower above.
- **Orientation Locking** – Cylinders stay perfectly upright to eliminate the common "laying down" bug from Gazebo.
- **TF Broadcasting** – Emits a dedicated TF frame per actor so planners, perception nodes, and RViz overlays stay synchronized.

## Architecture
| Stage | Description |
| --- | --- |
| **Input** | `/model_states` (`gazebo_msgs/ModelStates`) from Gazebo Classic |
| **Logic** | Regex filter (`human_*`) → planar velocity magnitude → marker + TF construction |
| **Output** | `/actor_markers` (`visualization_msgs/MarkerArray`) for RViz plus `/tf` transforms |

## Installation & Build
```bash
# From your ROS 2 workspace
cd ~/ros2_ws
colcon build --packages-select gazebo_actor_relay
source install/setup.bash
```

## Usage
Make sure Gazebo Classic is running with actors already spawned. Then start the relay:
```bash
ros2 run gazebo_actor_relay gazebo_actor_relay
```
Open RViz2, add a MarkerArray display for `/actor_markers`, and TF will automatically stream.

## Configuration / Parameters
| Name | Default | Description |
| --- | --- | --- |
| `planner_frame` | `"world"` | Frame ID used for marker headers and TF parent frames. |

## Dependencies
- `rclpy`
- `gazebo_msgs`
- `visualization_msgs`
- `geometry_msgs`
- `tf2_ros`

