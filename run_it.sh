#!/bin/bash
# build_and_run.sh
# Optional cleanup, build, source, and run script for gazebo_actor_spawner

set -e  # Exit immediately if a command fails

echo "=== Cleaning previous build (optional) ==="
rm -rf build install log || true

echo "=== Building workspace ==="
colcon build --symlink-install

echo "=== Sourcing setup file ==="
source install/setup.bash

echo "=== Running gazebo_actor_spawner ==="
ros2 run gazebo_actor_spawner spawn_actors
