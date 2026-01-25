#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import math

class CrazyflieMarkerPublisher(Node):
    def __init__(self):
        super().__init__('crazyflie_marker_publisher')
        
        # Declare parameters
        self.declare_parameter('rigid_body_name', 'cf1')
        self.declare_parameter('mesh_path', 'package://crazyflie_viz/gui/cf2_assembly.stl')
        self.declare_parameter('mesh_scale', 3.0)
        
        # Get parameters
        rigid_body_name = self.get_parameter('rigid_body_name').value
        self.mesh_path = self.get_parameter('mesh_path').value
        mesh_scale = self.get_parameter('mesh_scale').value
        
        # Subscribe to OptiTrack pose
        optitrack_topic = f'/optitrack/{rigid_body_name}/pose'
        self.subscription = self.create_subscription(
            PoseStamped,
            optitrack_topic,
            self.pose_callback,
            10
        )
        
        # Publish marker for RViz
        self.marker_pub = self.create_publisher(Marker, '/crazyflie/marker', 10)
        
        # Store mesh scale
        self.mesh_scale = mesh_scale
        
        self.get_logger().info(f"✅ Crazyflie Marker Publisher node started")
        self.get_logger().info(f"📡 Subscribing to: {optitrack_topic}")
        self.get_logger().info(f"📊 Publishing markers to: /crazyflie/marker")
    
    def pose_callback(self, msg: PoseStamped):
        """
        Callback for OptiTrack PoseStamped messages.
        OptiTrack already provides position and orientation as quaternion.
        """
        marker = Marker()
        marker.header = msg.header  # Use the same header from OptiTrack
        marker.ns = "crazyflie"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        
        # Mesh path
        marker.mesh_resource = self.mesh_path
        marker.mesh_use_embedded_materials = False
        
        # Position - directly from OptiTrack
        marker.pose.position = msg.pose.position
        
        # Orientation - directly from OptiTrack (already quaternion)
        # OptiTrack provides orientation as quaternion (x, y, z, w)
        marker.pose.orientation = msg.pose.orientation
        
        # Appearance
        marker.scale.x = self.mesh_scale
        marker.scale.y = self.mesh_scale
        marker.scale.z = self.mesh_scale
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()