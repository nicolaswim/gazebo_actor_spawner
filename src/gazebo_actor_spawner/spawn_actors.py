#!/usr/bin/env python3
import sys
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import time

ACTOR_TEMPLATE = """
    <actor name="human{actor_id}">
      <skin>
        <filename>model://walking_human/meshes/moonwalk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>model://walking_human/meshes/walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <waypoint>
            <time>0</time>
            <pose>{pose}</pose>
          </waypoint>
          <waypoint>
            <time>{waypoint_time}</time>
            <pose>{waypoint_pose}</pose>
          </waypoint>
          <waypoint>
            <time>{waypoint_time_return}</time>
            <pose>{waypoint_pose_return}</pose>
          </waypoint>
          <waypoint>
            <time>{waypoint_time_total}</time>
            <pose>{pose}</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>
"""

DIRECTIONS = {
    "1": (-15, 0, 0, 15, 0), "2": (15, 0, 3.14, -15, 0),
    "3": (0, -15, 1.57, 0, 15), "4": (0, 15, -1.57, 0, -15)
}
PATH_LENGTH = 30.0

def get_user_input(prompt, valid_options=None):
    while True:
        val = input(prompt).strip()
        if not valid_options or val in valid_options: return val
        print(f"Invalid input. Please choose one of: {valid_options}")

def generate_actor_sdf(actor_id, direction_key, speed):
    start_x, start_y, start_yaw, end_x, end_y = DIRECTIONS[direction_key]
    initial_pose_str = f"{start_x} {start_y} 0 0 0 {start_yaw}"
    end_pose_str = f"{end_x} {end_y} 0 0 0 {start_yaw}"
    time_to_dest = PATH_LENGTH / speed
    time_to_dest_return = time_to_dest * 2 + 1.0
    time_total = time_to_dest * 2 + 2.0
    return ACTOR_TEMPLATE.format(
        actor_id=actor_id, pose=initial_pose_str,
        waypoint_time=f"{time_to_dest:.2f}", waypoint_pose=end_pose_str,
        waypoint_time_return=f"{time_to_dest_return:.2f}",
        waypoint_pose_return=f"{start_x} {start_y} 0 0 0 {start_yaw + 3.14159}",
        waypoint_time_total=f"{time_total:.2f}"
    )

def main():
    print("--- Gazebo Actor Spawner ---")
    try:
        num_humans_str = get_user_input("How many humans would you like to simulate? (1-50) ")
        num_humans = int(num_humans_str)
        if not 1 <= num_humans <= 50: raise ValueError
    except ValueError:
        print("Invalid number. Exiting."); sys.exit(1)
    except KeyboardInterrupt:
        print("\nCancelled by user. Exiting."); sys.exit(1)

    actor_xml_list = []
    try:
        for i in range(1, num_humans + 1):
            print(f"\n--- Configuring Human {i} (TF name: human_{i-1}) ---")
            dir_key = get_user_input("Direction? (1=East, 2=West, 3=North, 4=South): ", ["1", "2", "3", "4"])
            speed_str = get_user_input("Speed in m/s? (e.g., 1.2): ")
            try:
                speed = float(speed_str)
                if speed <= 0.1 or speed > 3.0: speed = 1.0; print("Unrealistic speed. Setting to 1.0 m/s.")
            except ValueError: speed = 1.0; print("Invalid speed. Setting to 1.0 m/s.")
            actor_xml_list.append(generate_actor_sdf(i, dir_key, speed))
    except KeyboardInterrupt:
        print("\nCancelled by user. Exiting."); sys.exit(1)

    print("\nGenerating world file...")
    try:
        pkg_share_dir = get_package_share_directory('gazebo_actor_spawner')
        template_path = os.path.join(pkg_share_dir, 'worlds', 'empty.sdf')
        with open(template_path, 'r') as f: world_template = f.read()
    except Exception as e:
        print(f"Error: Could not find template world file at {template_path}"); sys.exit(1)
        
    all_actors_xml = "\n".join(actor_xml_list)
    final_world_sdf = world_template.replace("", all_actors_xml)
    temp_world_path = "/tmp/generated_actor_world.sdf"
    with open(temp_world_path, 'w') as f: f.write(final_world_sdf)
        
    print(f"World file saved to {temp_world_path}")
    print("Launching simulation in 5 seconds... (Press Ctrl+C in this terminal to stop)")
    time.sleep(5)

    launch_command = [
        "ros2", "launch",
        "gazebo_actor_spawner", "view_actors.launch.py",
        f"world_path:={temp_world_path}"
    ]
    try:
        subprocess.run(launch_command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Launch command failed: {e}")
    except KeyboardInterrupt:
        print("\nSimulation launch interrupted by user.")
    finally:
        print("Simulation shut down.")

if __name__ == "__main__":
    main()