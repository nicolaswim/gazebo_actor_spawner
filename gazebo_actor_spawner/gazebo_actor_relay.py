#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, TransformStamped
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
import re

class GazeboActorRelay(Node):
    def __init__(self):
        super().__init__('gazebo_actor_relay')
        
        self.declare_parameter('planner_frame', 'world')
        self.planner_frame = self.get_parameter('planner_frame').value
        
        # Matches 'human_1', 'human_2' from the new spawner script
        self.actor_regex = re.compile(r"human_\d+")
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/actor_markers', 10)
        
        self.model_states_sub = self.create_subscription(
            ModelStates, '/model_states', self.model_states_callback, QoSProfile(depth=10)
        )
        
        self.get_logger().info(f"Relay Started. Listening for 'human_X'...")

    def model_states_callback(self, msg):
        now = self.get_clock().now()
        marker_array = MarkerArray()
        transforms = []

        # 1. DELETE OLD MARKERS (Clear previous frame)
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = self.planner_frame
        marker_array.markers.append(delete_marker)

        try:
            name_to_index = {name: i for i, name in enumerate(msg.name)}
        except:
            return

        count = 0
        for actor_name in name_to_index.keys():
            if not self.actor_regex.match(actor_name):
                continue
            
            count += 1
            index = name_to_index[actor_name]
            pose = msg.pose[index]

            # 2. VISUAL MARKER (Green Box)
            marker = Marker()
            marker.header.frame_id = self.planner_frame
            marker.header.stamp = now.to_msg()
            marker.ns = "humans"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale = Vector3(x=0.5, y=0.5, z=1.8)
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            marker.lifetime = Duration(seconds=0.2).to_msg()
            marker_array.markers.append(marker)

            # 3. TF TRANSFORM
            # We map human_1 -> human_1 directly (No name changing)
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.planner_frame
            t.child_frame_id = actor_name 
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation
            transforms.append(t)

        # Publish
        if count > 0:
            self.marker_array_pub.publish(marker_array)
            self.tf_broadcaster.sendTransform(transforms)
            # DEBUG: Uncomment the line below if you want to see spam in the console
            # self.get_logger().info(f"Published TF for {count} humans")

def main(args=None):
    rclpy.init(args=args)
    node = GazeboActorRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()