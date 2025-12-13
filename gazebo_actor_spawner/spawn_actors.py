#!/usr/bin/env python3
import sys
import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def generate_world_file(filepath):
    """
    Generates a basic SDF world file so Gazebo has something to load.
    **IMPORTANT:** Restore your specific actor/trajectory generation logic here.
    """
    sdf_content = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="actor_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    </world>
</sdf>
"""
    with open(filepath, 'w') as f:
        f.write(sdf_content)

def main():
    # 1. Define the temporary path explicitly
    temp_world_path = "/tmp/generated_actor_world.sdf"

    # 2. Generate the world file (so Gazebo doesn't fail loading a missing file)
    # Restore your user input logic here if needed
    print(f"Generating world at {temp_world_path}...")
    generate_world_file(temp_world_path)
    
    print(f"World saved. Launching...")

    # 3. Construct the command
    cmd = [
        "ros2", "launch", "gazebo_actor_spawner", "view_actors.launch.py",
        f"world_path:={temp_world_path}"
    ]

    # 4. Execute with full system environment (Launch file handles isolation)
    try:
        subprocess.run(cmd, check=True, env=os.environ)
    except KeyboardInterrupt:
        pass
    except subprocess.CalledProcessError as e:
        print(f"Process failed with error: {e}")

if __name__ == "__main__":
    main()