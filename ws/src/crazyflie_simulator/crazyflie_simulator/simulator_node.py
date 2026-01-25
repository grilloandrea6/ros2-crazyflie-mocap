#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from crazyflie_msgs.msg import CrazyflieControlStamped, CrazyflieStateStamped
from crazyflie_model.crazyfliemodel import CrazyFlieModel
import numpy as np


class CrazyFlieSimulator(Node):
    def __init__(self):
        super().__init__('crazyflie_simulator')

        qos = QoSProfile(depth=10)

        # Parameters
        self.declare_parameter('rate', 300.0)  # simulation frequency [Hz]
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.dt = 1.0 / self.rate

        # Initialize dynamics model
        self.model = CrazyFlieModel(freq=self.rate)

        # Subscribers and publishers
        self.control_sub = self.create_subscription(
            CrazyflieControlStamped,
            '/crazyflie/control',
            self.control_callback,
            qos
        )

        self.state_pub = self.create_publisher(
            CrazyflieStateStamped,
            '/crazyflie/state',
            qos
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/optitrack/cf1/pose',
            qos
        )

        # Timer for integration
        self.timer = self.create_timer(self.dt, self.update)

        # Control input
        self.last_control = np.copy(self.model.hover_thrust)

        # Initial state
        self.state = np.zeros(self.model.nx)
        self.state[0] = 1.0
        self.state[2] = 1.0

        self.get_logger().info("✅ CrazyFlie Simulator started (dt = %.3f s)" % self.dt)

    def control_callback(self, msg: CrazyflieControlStamped):
        u = np.array([msg.motor_1, msg.motor_2, msg.motor_3, msg.motor_4])
        self.last_control = u

    def update(self):
        # Integrate one step of dynamics
        self.state = self.model.step(self.state, self.last_control)

        if any(np.isnan(self.state)) or any(np.abs(self.state[0:3]) > 10.0):
            self.get_logger().error("Simulator exploded! Resetting to zero state and hover input.")
            self.state = np.zeros(self.model.nx)
            self.state[0] = 1.0
            self.state[2] = 1.0

            self.last_control = np.copy(self.model.hover_thrust)

        
        # Publish state
        msg = CrazyflieStateStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.state_msg.state = [float(x) for x in self.state]
        self.state_pub.publish(msg)

        # Publish PoseStamped (position + orientation from state)
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position.x = float(self.state[0])
        pose_msg.pose.position.y = float(self.state[1])
        pose_msg.pose.position.z = float(self.state[2])
        q = self.model.rptoq(self.state[3:6])
        pose_msg.pose.orientation.x = float(q[1])
        pose_msg.pose.orientation.y = float(q[2])
        pose_msg.pose.orientation.z = float(q[3])
        pose_msg.pose.orientation.w = float(q[0])
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CrazyFlieSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
