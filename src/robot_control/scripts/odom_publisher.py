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
from std_msgs.msg import Header, Float32, Float32MultiArray
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import math
import numpy as np


class odom_publisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")

        self.dt_loop = 0.01

        self.publisher = self.create_publisher(Odometry, 'odom', 10)

        self.subscriptions_wheelspeed = self.create_subscription(Float32MultiArray, 'feedback_wheelspeed', self.wheelspeed_callback, 10)
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)
        self.tf_br = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_th = 0.0

        self.rightwheel_speed = 0.0
        self.leftwheel_speed = 0.0

    def wheelspeed_callback(self, msg):
        current_time = self.get_clock().now()
        if not hasattr(self, 'last_callback_time'):
            self.last_callback_time = current_time
            return
        dt = (current_time - self.last_callback_time).to_msg().nanosec * 1e-9

        self.last_callback_time = current_time
        self.leftwheel_speed = msg.data[0] * 0.075
        self.rightwheel_speed = msg.data[1] * 0.075

        self.delta_x = (self.rightwheel_speed + self.leftwheel_speed) * 0.5 * math.cos(self.th)
        self.delta_y = (self.rightwheel_speed + self.leftwheel_speed) * 0.5 * math.sin(self.th)
        self.delta_th = (self.rightwheel_speed - self.leftwheel_speed) / 0.4

        self.x += np.round(self.delta_x * dt, 3)
        self.y += np.round(self.delta_y * dt, 3)
        self.th += np.round(self.delta_th * dt, 3)

    def timer_callback(self):
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
        odom_msg.twist.twist.linear = Vector3(x=np.round(self.delta_x, 3), y=np.round(self.delta_y, 3), z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=np.round(self.delta_th, 3))
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

    
def main(args=None):
    rclpy.init(args=args)
    node = odom_publisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
