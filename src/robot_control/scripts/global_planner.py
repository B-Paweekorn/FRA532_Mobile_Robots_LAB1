#!/usr/bin/env python3

from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import tf_transformations
import numpy as np
import matplotlib.pyplot as plt

class GlobalCostmapNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.map_callback, 10)
        self.create_subscription(Odometry, 'odom', self.robotpose_callback, 10)
        self.create_timer(0.05, self.update)
        
        self.visualize = True
        self.map_received = False
        self.map_data = None
        self.map_info = None
        
        # robot pose [m]
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_th = 0.0

        # robot goal [m]
        self.goal_x = 0.0
        self.goal_y = 0.0
        
        # robot properties [m]
        self.robot_radius = 5.0

    def robotpose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_th = tf_transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = msg.data

    def update(self):
        self.map, self.ox, self.oy = self.create_map()
        if self.map_received is True:
            self.vff_planner()
            if self.visualize is True:
                print("Start visualizing map")
                plt.imshow(self.map, cmap='gray', extent=[self.ox[0], self.ox[-1], self.oy[0], self.oy[-1]])
                plt.colorbar()
                plt.title('Occupancy Grid Map')
                plt.xlabel('X Coordinate (m)')
                plt.ylabel('Y Coordinate (m)')
                plt.show()

    def create_map(self):
        if self.map_info is None:
            self.get_logger().warn('Waiting for map.')
            self.map_received = False
            return None, None, None
        print("Received map")
        width = self.map_info.width
        height = self.map_info.height
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        map_array = np.array(self.map_data).reshape(height, width)
        
        filtercost_map = np.where(map_array < 100, 0, map_array)
        filtercost_map = np.flipud(filtercost_map)

        map_array_x = np.linspace(origin_x, origin_x + resolution * width, width)
        map_array_y = np.linspace(origin_y, origin_y + resolution * height, height)
        
        self.map_received = True
        return filtercost_map, map_array_x, map_array_y

    def vff_planner(self):
        pass

    def path_marker(self, path):
        for i in range(len(path)):
            # Publish the goal point for visualization in RViz
            goal_marker = PoseStamped()
            goal_marker.header.frame_id = "map"
            goal_marker.pose.position = path[i].pose.position
            self.goal_marker_publisher.publish(goal_marker)
            return path[i]
        return None
        

def main(args=None):
    rclpy.init(args=args)
    global_costmap_node = GlobalCostmapNode()
    rclpy.spin(global_costmap_node)
    global_costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
