#!/usr/bin/env python3
import sys
import os
import subprocess
import math

def generate_linear_trajectory(x_start, y_start, direction_deg, speed, duration=20.0):
    waypoints = []
    t = 0.0
    dt = 0.5 
    
    yaw = math.radians(direction_deg)
    vel_x = speed * math.cos(yaw)
    vel_y = speed * math.sin(yaw)

    current_x = x_start
    current_y = y_start

    while t <= duration:
        waypoints.append(f"""
           <waypoint>
              <time>{t:.2f}</time>
              <pose>{current_x:.2f} {current_y:.2f} 0 0 0 {yaw:.2f}</pose>
           </waypoint>""")
        
        current_x += vel_x * dt
        current_y += vel_y * dt
        t += dt

    return "".join(waypoints)

def generate_actor_xml(name, x, y, direction, speed):
    trajectory_xml = generate_linear_trajectory(x, y, direction, speed)
    
    skin_file = "model://walking_human/meshes/moonwalk.dae"
    anim_file = "model://walking_human/meshes/walk.dae"
    
    return f"""
    <actor name="{name}">
      <pose>{x} {y} 0 0 0 0</pose>
      <skin>
        <filename>{skin_file}</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>{anim_file}</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
           {trajectory_xml}
        </trajectory>
      </script>
    </actor>
    """

def main():
    print("\n=== Gazebo Actor Spawner (World Plugin Fix) ===")
    
    try:
        # 1. Ask for the amount of humans ONCE
        num_humans = int(input("Amount of actors? (e.g. 3): "))
    except ValueError:
        print("Invalid input. Defaulting to 1 actor.")
        num_humans = 1

    actors_xml = []
    
    # 2. Loop through the amount and ask details for EACH human
    for i in range(num_humans):
        print(f"\n--- Configuration for Human {i+1} ---")
        try:
            # We ask for direction and speed inside the loop now
            direction_deg = float(input(f"Direction for Human {i+1} (0=East, 90=North): "))
            speed = float(input(f"Speed for Human {i+1} (m/s): "))
        except ValueError:
            print(f"Invalid input for Human {i+1}. Using defaults: 0 deg, 1.0 m/s.")
            direction_deg = 0.0
            speed = 1.0
            
        start_x = 0.0
        start_y = i * 2.0 
        
        # Generate the XML using these specific values
        xml = generate_actor_xml(f"human_{i+1}", start_x, start_y, direction_deg, speed)
        actors_xml.append(xml)

    joined_actors = "\n".join(actors_xml)
    
    debug_box = """
    <model name="debug_box">
      <pose>-2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
          <material><script><name>Gazebo/Red</name></script></material>
        </visual>
      </link>
    </model>
    """

    ros_state_plugin = """
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/</namespace>
        <remapping>model_states:=model_states</remapping>
      </ros>
      <update_rate>10.0</update_rate>
    </plugin>
    """

    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="actor_world">
    {ros_state_plugin}
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    {debug_box}
    {joined_actors}
  </world>
</sdf>
"""

    temp_world_path = "/tmp/generated_actor_world.sdf"
    with open(temp_world_path, 'w') as f:
        f.write(sdf_content)

    print(f"\nSaved world to {temp_world_path}")
    print("Launching...")

    cmd = [
        "ros2", "launch", "gazebo_actor_spawner", "view_actors.launch.py",
        f"world_path:={temp_world_path}"
    ]

    try:
        subprocess.run(cmd, check=True, env=os.environ)
    except KeyboardInterrupt:
        pass
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()