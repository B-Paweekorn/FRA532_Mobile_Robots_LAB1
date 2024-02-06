#!/usr/bin/env python3

from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

class GlobalCostmapNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.map_callback, 10)

        self.create_timer(0.05, self.update)
        
        self.map_received = False
        self.map_data = None
        self.map_info = None

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0

        
    def map_callback(self, msg):
        # Process the received map
        self.map_info = msg.info
        self.map_data = msg.data

    def update(self):
        if self.map_data is not None and not self.map_received:
            self.map_received = True
            print("Received map !!!")
            self.visualize_map()
        else:
            print("Waiting map...")
    def visualize_map(self):
        if self.map_info is None:
            self.get_logger().warn('No map info received.')
            return

        # Extract width and height from map info
        width = self.map_info.width
        height = self.map_info.height
        
        # Convert map data to numpy array
        map_array = np.array(self.map_data).reshape(height, width)
        
        # Extract origin position from map info
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        # Create x and y coordinates for the map grid
        x_coords = np.linspace(origin_x, origin_x + self.map_info.resolution * width, width)
        y_coords = np.linspace(origin_y, origin_y + self.map_info.resolution * height, height)
        
        # Plot the map
        plt.imshow(map_array, cmap='gray', extent=[x_coords[0], x_coords[-1], y_coords[0], y_coords[-1]])
        plt.colorbar()
        plt.title('Occupancy Grid Map')
        plt.xlabel('X Coordinate (m)')
        plt.ylabel('Y Coordinate (m)')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    global_costmap_node = GlobalCostmapNode()
    rclpy.spin(global_costmap_node)
    global_costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
