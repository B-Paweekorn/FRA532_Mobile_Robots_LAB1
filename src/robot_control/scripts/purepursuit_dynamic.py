#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
import tf_transformations
import math
import numpy as np
import time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Pose, TransformStamped


class DifferentialDrivePurePursuit(Node):
    def __init__(self):
        super().__init__("differential_drive_pure_pursuit")
        self.create_subscription(PointStamped, "clicked_point", self.clicked_point_callback, 10)
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel_purepursuit', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.pure_pursuit_controller)

        self.lookahead_distance = 0.6
        self.goal_threshold = 0.6

        self.controller_active = 0
        self.send_goal_in_progress = False
        self.goal_pose = None
        self.robot_pose = None
        self.path = None
        self.current_pose_index = 0
        self.next_update_time = 0

    def pure_pursuit_controller(self):
        self.robot_pose = self.get_robot_pose()
        if self.robot_pose is None or self.goal_pose is None:
            # do not run until robot pose is ready and goal is set
            return
        if self.controller_active != 1:
            # do not run if controller is not active
            return
        if self.is_goal_reached(self.robot_pose, self.goal_pose):
            # disable controller and stop robot once goal is reached
            self.controller_active = 0
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info('Goal reached')
            return
        if self.next_update_time < time.time() and not self.send_goal_in_progress:
            self.next_update_time = time.time() + 0.0
            # get new path and reset current pose index
            self.send_goal_in_progress = True
            self.send_goal(self.goal_pose)
            self.current_pose_index = 0

        # main logic
        if self.current_pose_index < len(self.path):
            current_pose = self.path[self.current_pose_index]
            # skip to the closest point in the path
            dx = []
            dy = []
            for i in range(len(self.path)):
                dx.append(self.robot_pose.position.x - self.path[i].pose.position.x)
                dy.append(self.robot_pose.position.y - self.path[i].pose.position.y)
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.current_pose_index = ind if ind > self.current_pose_index else self.current_pose_index
            # calculate the next goal point
            goal_point = self.calculate_goal_point(self.path, self.robot_pose, self.current_pose_index)
            if goal_point is not None:
                linear_vel, angular_vel = self.calculate_velocities(self.robot_pose, goal_point)
                self.publish_velocity(linear_vel, angular_vel)

    def calculate_goal_point(self, path, robot_pose, start_index):
        for i in range(start_index, len(path)):
            if self.distance_between_points(robot_pose.position, path[i].pose.position) >= self.lookahead_distance:
                return path[i]
        return None
    
    def calculate_velocities(self, robot_pose, goal_point):
        max_linear_velocity = 0.50  # Maximum linear velocity
        max_angular_velocity = 0.60  # Maximum angular velocity
        # Calculate the angle to the goal point
        angle_to_goal = math.atan2(goal_point.pose.position.y - robot_pose.position.y,
                                   goal_point.pose.position.x - robot_pose.position.x)
        # Convert the robot's orientation from quaternion to Euler angles
        _, _, yaw = tf_transformations.euler_from_quaternion([robot_pose.orientation.x,
                                                              robot_pose.orientation.y,
                                                              robot_pose.orientation.z,
                                                              robot_pose.orientation.w])
        # Calculate the heading error
        heading_error = self.normalize_angle(angle_to_goal - yaw)
        # Calculate the linear and angular velocities
        linear_velocity = max_linear_velocity * (1 - abs(heading_error))
        angular_velocity = max_angular_velocity * heading_error
        return linear_velocity, angular_velocity
    
    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2.0 * math.pi
        if angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clicked_point_callback(self, msg):
        self.controller_active = 2
        self.send_goal_in_progress = False
        self.get_logger().info(f"Received goal point: [{msg.point.x:.2}, {msg.point.y:.2}]")
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.pose.position.x = msg.point.x
        self.goal_pose.pose.position.y = msg.point.y
        self.send_goal(self.goal_pose)

    def send_goal(self, pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose
        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.send_goal_in_progress = False
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Path received: len[{len(result.path.poses)}]')
        self.path = result.path.poses
        self.send_goal_in_progress = False
        if self.controller_active == 2:
            self.controller_active = 1

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error('Could not transform from base_link to map: %s' % str(e))
            return None
        
    def publish_velocity(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.velocity_publisher.publish(twist)

    def is_goal_reached(self, robot_pose, goal_pose):
        return self.distance_between_points(robot_pose.position, goal_pose.pose.position) <= self.goal_threshold

    def distance_between_points(self, point1, point2):
        return math.hypot(point1.x - point2.x, point1.y - point2.y)


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDrivePurePursuit()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
