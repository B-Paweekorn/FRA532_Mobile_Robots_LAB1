#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import (
    PoseStamped, Twist, Point, Pose, PoseWithCovariance, Quaternion,
    TwistWithCovariance, Vector3, TransformStamped
)
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float32
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import numpy as np


class WheelController(Node):
    def __init__(self):
        super().__init__("wheel_controller")

        self.dt_loop = 0.01

        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)
        self.tf_br = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0


    def timer_callback(self):
        # delta_x = self.wheelspeed * math.cos(self.relative_yaw) * self.dt_loop
        # delta_y = self.wheelspeed * math.sin(self.relative_yaw) * self.dt_loop

        delta_x = 0.0
        delta_y = 0.0

        self.x += delta_x
        self.y += delta_y

        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.th)
        # Create Odometry message and fill in the data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()  # Update time stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        )
        odom_msg.twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.publisher.publish(odom_msg)

        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_br.sendTransform(transform)

        # transform = TransformStamped()
        # transform.header.stamp = odom_msg.header.stamp
        # transform.header.frame_id = 'map'
        # transform.child_frame_id = 'odom'
        # transform.transform.translation.x = 0.0
        # transform.transform.translation.y = 0.0
        # transform.transform.translation.z = 0.0
        # transform.transform.rotation = 0.0
        # self.tf_br.sendTransform(transform)

    
def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
