#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist
import tf_transformations
import math
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import LaserScan

"""
LaserScan start counting 0 from the back of the robot, and goes anticlockwise.
"""


class VirtualForceField(Node):
    def __init__(self):
        super().__init__("virtual_force_field")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.create_timer(0.05, self.VFF_controller)
        self.min_rad = 0.0
        self.step_rad = 0.0
        self.min_distance = 10.0
        self.min_distance_rad = 0.0

    def VFF_controller(self):
        cmd = [1.0, 0.0]
        if(self.min_distance < 2.0):
            k = max(1.0 - self.min_distance, 0)
            cmd[1] += 2 * k * (self.min_distance_rad + math.pi)
        self.pub_cmd_vel(cmd)

    def pub_cmd_vel(self, cmd):
        twist = Twist()
        twist.linear.x = float(cmd[0])
        twist.angular.z = float(cmd[1])
        self.cmd_vel_pub.publish(twist)

    def scan_callback(self, msg):
        self.min_rad = msg.angle_min
        self.step_rad = msg.angle_increment
        range_array = np.array(msg.ranges)
        self.min_distance = np.min(range_array)
        self.min_distance_rad = self.min_rad + np.argmin(range_array) * self.step_rad
        print(self.min_distance_rad)

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error("Could not transform from base_link to map: %s" % str(e))
            return None


def main(args=None):
    rclpy.init(args=args)
    node = VirtualForceField()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
