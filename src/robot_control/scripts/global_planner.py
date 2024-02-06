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
        
        self.visualize = True
        self.map_received = False
        self.map_data = None
        self.map_info = None
        
        # robot pose [m]
        self.robot_x = 0.0
        self.robot_y = 0.0

        # robot goal [m]
        self.goal_x = 0.0
        self.goal_y = 0.0
        
        # robot properties [m]
        self.robot_radius = 5.0

    def map_callback(self, msg):
        # Process the received map
        self.map_info = msg.info
        self.map_data = msg.data

    def update(self):
        self.map, self.ox, self.oy = self.create_map()
        if self.map_received is True:
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
        map_array_x = np.linspace(origin_x, origin_x + resolution * width, width)
        map_array_y = np.linspace(origin_y, origin_y + resolution * height, height)
        
        self.map_received = True
        return map_array, map_array_x, map_array_y


def main(args=None):
    rclpy.init(args=args)
    global_costmap_node = GlobalCostmapNode()
    rclpy.spin(global_costmap_node)
    global_costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
