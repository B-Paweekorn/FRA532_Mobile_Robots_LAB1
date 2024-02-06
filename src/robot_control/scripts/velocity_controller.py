#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import (
    PoseStamped, Twist, Point, Pose, PoseWithCovariance, Quaternion,
    TwistWithCovariance, Vector3, TransformStamped
)
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float32, Float64MultiArray, Float32MultiArray
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import math
import numpy as np


class VelocityController(Node):
    def __init__(self):
        super().__init__("velocity_controller")

        self.wheel_radius = 0.075
        self.wheel_separation = 0.4

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.pub_wheelspeed = self.create_publisher(Float32MultiArray, "feedback_wheelspeed", 10)
        self.pub_velocities = self.create_publisher(Float64MultiArray, "velocity_controllers/commands", 10)

    
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        meter2rev = 1 / (2 * math.pi * self.wheel_radius)

        # convert to wheel speeds of differential drive robot (rev/s)
        left_wheel_speed = linear * meter2rev - (angular * self.wheel_separation / 2) * meter2rev
        right_wheel_speed = linear * meter2rev + (angular * self.wheel_separation / 2) * meter2rev

        # convert wheel speeds to angular velocities (rad/s)
        left_wheel_speed *= 2 * math.pi
        right_wheel_speed *= 2 * math.pi

        # send wheel speeds to robot
        msg = Float64MultiArray()
        msg.data = [left_wheel_speed, right_wheel_speed]
        self.pub_velocities.publish(msg)

    def joint_state_callback(self, msg):
        # Check if the 'left_wheel_joint' and 'right_wheel_joint' are in the message
        if 'left_wheel_joint' in msg.name and 'right_wheel_joint' in msg.name:
            left_wheel_index = msg.name.index('left_wheel_joint')
            right_wheel_index = msg.name.index('right_wheel_joint')

            # Extract wheel velocities
            left_wheel_velocity = msg.velocity[left_wheel_index]
            right_wheel_velocity = msg.velocity[right_wheel_index]

            msg = Float32MultiArray()
            msg.data = [left_wheel_velocity, right_wheel_velocity]
            self.pub_wheelspeed.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
