# gazebo_actor_relay
> Bridging Gazebo Classic actors into RViz2 with live markers and TF updates.

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue) ![License: MIT](https://img.shields.io/badge/License-MIT-green)

## Gallery
| Gazebo Classic (Source) | RViz2 (Visualization) |
| :---: | :---: |
| ![Gazebo View](visuals/gazebo.png) | ![RViz View](visuals/rviz.png) |
> *Left: Animated actors in Gazebo. Right: The same actors rendered as dynamic cylinders in RViz.*

## Overview
Gazebo Classic can spawn animated humans, but RViz2 has no native way to render those actors or follow their transforms. `gazebo_actor_relay` closes that gap by listening to `/model_states`, filtering out `human_*` actors, and publishing both visualization markers and TF transforms. This allows operators to visualize pedestrian traffic and obstacles in RViz just like any other robot component.

## Key Features
- **Dynamic Visuals** – Every actor is rendered as an easily visible **green cylinder**, ensuring high contrast against map data.
- **Velocity-Based Scaling** – Marker height provides immediate visual feedback on speed. Stationary actors appear as 1.0 m markers, while moving actors grow taller based on velocity (`height = sqrt(vx^2 + vy^2)`), allowing operators to instinctively gauge motion in the costmap.
- **Orientation Locking** – Forces marker orientation to align with the World Z-axis, correcting the common simulation bug where actors appear to "lay down" or slide horizontally.
- **TF Broadcasting** – Publishes a dedicated TF frame for every actor (using the actor's name, e.g., `human_1`) so planners and perception nodes can track them directly.

## Architecture
| Stage | Description |
| --- | --- |
| **Input** | `/model_states` (`gazebo_msgs/ModelStates`) from Gazebo Classic |
| **Logic** | Regex filter (`human_*`) → planar velocity magnitude → marker + TF construction |
| **Output** | `/actor_markers` (`visualization_msgs/MarkerArray`) for RViz plus `/tf` transforms |

```mermaid
graph TD
    subgraph Phase_1_Configuration [Phase 1: Procedural Generation]
        User[User Input<br>Count/Speed/Direction] --> Gen[spawn_actors.py<br>XML Assembly]
        Gen --> Write[File I/O<br>Write /tmp/generated_actor_world.sdf]
    end

    subgraph Phase_2_Simulation [Phase 2: Simulation Boot]
        Write --> Launch[ros2 launch<br>view_actors.launch.py]
        Launch --> GZ[Gazebo Server<br>Loads .sdf from /tmp]
    end

    subgraph Phase_3_Runtime [Phase 3: Semantic Injection]
        GZ -- /model_states --> Relay[gazebo_actor_relay]
        Relay -- Logic: Regex Filter --> Rect[State Rectification]
        Rect -- /tf --> TF[Navigation Stack]
        Rect -- /actor_markers --> RViz[Visualization]
    end
```

## Installation & Build
```bash
# From your ROS 2 workspace
cd ~/ros2_ws
colcon build --packages-select gazebo_actor_spawner
source install/setup.bash