#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Quaternion
from std_msgs.msg import ColorRGBA
import tf2_ros
from tf2_ros import TransformBroadcaster
import re
import math

class GazeboActorRelay(Node):
    def __init__(self):
        super().__init__('gazebo_actor_relay')
        self.declare_parameter('planner_frame', 'odom')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('unreliable_vel_threshold', 0.01)
        self.planner_frame = self.get_parameter('planner_frame').value
        self.publish_period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.vel_threshold = self.get_parameter('unreliable_vel_threshold').value
        self.actor_regex = re.compile(r"human\d+|wheelchair\d+")
        self.marker_namespace = "gazebo_actors"
        self.gazebo_to_planner_tf_map = {f"human{i}": f"human_{i-1}" for i in range(1, 51)}
        self.get_logger().info(f"Using dynamic TF map for 'human1' -> 'human_0', etc.")
        self.last_pose_cache = {}
        self.last_time_cache = {}
        self.latest_model_states = None
        gazebo_qos_profile = QoSProfile(depth=10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/actor_markers', 10)
        self.model_states_sub = self.create_subscription(ModelStates, '/model_states', self.model_states_callback, gazebo_qos_profile)
        self.publish_timer = self.create_timer(self.publish_period, self.process_and_publish)
        self.get_logger().info("General Gazebo Actor Relay started.")

    def model_states_callback(self, msg):
        self.latest_model_states = msg

    def is_velocity_unreliable(self, vel):
        return abs(vel.x) < self.vel_threshold and abs(vel.y) < self.vel_threshold

    def process_and_publish(self):
        if self.latest_model_states is None: return
        msg, now = self.latest_model_states, self.get_clock().now()
        marker_array_msg, transforms_list = MarkerArray(), []
        clear_marker = Marker()
        clear_marker.header.frame_id, clear_marker.header.stamp = self.planner_frame, now.to_msg()
        clear_marker.ns, clear_marker.action = self.marker_namespace, Marker.DELETEALL
        marker_array_msg.markers.append(clear_marker)
        try: name_to_index = {name: i for i, name in enumerate(msg.name)}
        except Exception as e: self.get_logger().error(f"Error creating name_to_index map: {e}"); return
        for actor_name in name_to_index.keys():
            if not self.actor_regex.match(actor_name): continue
            if actor_name not in self.gazebo_to_planner_tf_map: continue
            planner_tf_name, index = self.gazebo_to_planner_tf_map[actor_name], name_to_index[actor_name]
            actor_pose = msg.pose[index]
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.planner_frame, now.to_msg()
            marker.ns, marker.id = self.marker_namespace, index
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            marker.pose = actor_pose
            marker.scale = Vector3(x=0.5, y=0.5, z=1.8)
            marker.color = ColorRGBA(r=0.5, g=0.8, b=1.0, a=0.8)
            marker.lifetime = Duration(seconds=self.publish_period * 2.0).to_msg()
            marker_array_msg.markers.append(marker)
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now.to_msg(), self.planner_frame, planner_tf_name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = actor_pose.position.x, actor_pose.position.y, actor_pose.position.z
            t.transform.rotation = actor_pose.orientation
            transforms_list.append(t)
        self.marker_array_pub.publish(marker_array_msg)
        self.tf_broadcaster.sendTransform(transforms_list)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboActorRelay()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()