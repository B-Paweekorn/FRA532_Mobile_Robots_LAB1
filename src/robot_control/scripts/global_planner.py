#!/usr/bin/env python3

from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import tf_transformations
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans


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
        self.goal_x = -11.0
        self.goal_y = 3.5
        
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
        self.map, self.scale_x, self.scale_y = self.create_map()
        if self.map_received is True:
            print("Received map")
            self.vff_planner(self.robot_x, self.robot_y, self.goal_x, self.goal_y, self.obs_x, self.obs_y, 0.3, 0.55)
            if self.visualize:
                print("Start visualizing map")
                fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

                ax1.imshow(self.map, cmap='gray', extent=[self.scale_x[0], self.scale_x[-1], self.scale_y[0], self.scale_y[-1]])
                ax1.set_title('Occupancy Grid Map')
                ax1.set_xlabel('X Coordinate (m)')
                ax1.set_ylabel('Y Coordinate (m)')
                ax1.grid(True)

                ax2.scatter(self.obs_x, self.obs_y, color='blue', marker='.')
                ax2.set_title('Obstacle Positions')
                ax2.set_xlabel('X Coordinate (m)')
                ax2.set_ylabel('Y Coordinate (m)')
                ax2.grid(True)

                ax2.set_xlim(ax1.get_xlim())
                ax2.set_ylim(ax1.get_ylim())

                plt.tight_layout()
                plt.show()

    def create_map(self):
        if self.map_info is None:
            self.get_logger().warn('Waiting for map.')
            self.map_received = False
            return None, None, None
        
        self.width = self.map_info.width
        self.height = self.map_info.height
        self.resolution = self.map_info.resolution
        self.origin_x = self.map_info.origin.position.x
        self.origin_y = self.map_info.origin.position.y

        map_array = np.array(self.map_data).reshape(self.height, self.width)
        
        filtercost_map = np.where(map_array < 100, 0, map_array)

        map_scale_x = np.linspace(self.origin_x, self.origin_x + self.resolution * self.width, self.width)
        map_scale_y = np.linspace(self.origin_y, self.origin_y + self.resolution * self.height, self.height)
        
        map_obstacle_x = []
        map_obstacle_y = []

        for i in range(self.height):
            for j in range(self.width):
                if filtercost_map[i][j] >= 100:
                    map_obstacle_x.append(map_scale_x[j])
                    map_obstacle_y.append(map_scale_y[i])

        filtercost_map = np.flipud(filtercost_map)

        obstacle_points = np.column_stack((map_obstacle_x, map_obstacle_y))
        kmeans = KMeans(n_clusters=400)  # Adjust the number of clusters as needed
        kmeans.fit(obstacle_points)
        cluster_centers = kmeans.cluster_centers_

        # Assign downsampled obstacle points to self.obs_x and self.obs_y
        self.obs_x = cluster_centers[:, 0]
        self.obs_y = cluster_centers[:, 1]
        print(len(map_obstacle_x))
        # self.obs_x = map_obstacle_x
        # self.obs_y = map_obstacle_y
        self.map_received = True
        return filtercost_map, map_scale_x, map_scale_y

    def vff_planner(self, sx, sy, gx, gy, ox, oy, reso, rbtr):
        # [sx, sy] : start point x, y
        # [gx, gy] : goal point x, y
        # [ox, oy] : obstacle x, y
        # rese : resolution
        # rbtr : robot radius
        # calc potential field
        print("Calculated potential map")
        pmap, minx, miny = self.vff_calc(gx, gy, ox, oy, reso, rbtr)
        print("Calculated potential map success")
        # search path
        d = np.hypot(sx - gx, sy - gy)
        ix = round((sx - minx) / reso)
        iy = round((sy - miny) / reso)
        gix = round((gx - minx) / reso)
        giy = round((gy - miny) / reso)

        if self.visualize:
            print("VFF started visualized")
            self.draw_heatmap(pmap)
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")
        
        rx, ry = [sx], [sy]
        motion = self.get_motion_model()

        while d >= reso:
            minp = float("inf")
            minix, miniy = -1, -1
            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")  # outside area
                    print("outside potential!")
                else:
                    p = pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix = minix
            iy = miniy
            xp = ix * reso + minx
            yp = iy * reso + miny
            d = np.hypot(gx - xp, gy - yp)
            rx.append(xp)
            ry.append(yp)

            if self.visualize:
                plt.plot(ix, iy, ".r")
                plt.pause(0.01)

        print("Goal!!")

        return rx, ry
    
    def vff_calc(self, gx, gy, ox, oy, reso, rbtr):
        minx = self.origin_x
        miny = self.origin_y
        maxx = self.origin_x + self.resolution * self.width
        maxy = self.origin_y + self.resolution * self.height
        xw = int(round((maxx - minx) / reso))
        yw = int(round((maxy - miny) / reso))

        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            x = ix * reso + minx
            print(ix*100/xw, "%")
            for iy in range(yw):
                y = iy * reso + miny
                ug = self.calc_attractive_potential(x, y, gx, gy)
                uo = self.calc_repulsive_potential(x, y, ox, oy, rbtr)
                uf = ug + uo
                pmap[ix][iy] = uf
        return pmap, minx, miny

    def calc_attractive_potential(self, x, y, gx, gy):
        return 0.5 * 20 * np.hypot(x - gx, y - gy)

    def calc_repulsive_potential(self, x, y, ox, oy, rbtr):
        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(ox):
            d = np.hypot(x - ox[i], y - oy[i])
            if dmin >= d:
                dmin = d
                minid = i

        dq = np.hypot(x - ox[minid], y - oy[minid])

        if dq <= rbtr:
            if dq <= 0.1:
                dq = 0.1
            return 0.5 * 3000.0 * (1.0 / dq - 1.0 / rbtr) ** 2
        else:
            return 0.0
    
    def draw_heatmap(self, data):
        data = np.array(data).T
        plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)
        
    def get_motion_model(self):
        # dx, dy
        motion = [[1, 0],
                [0, 1],
                [-1, 0],
                [0, -1],
                [-1, -1],
                [-1, 1],
                [1, -1],
                [1, 1]]
        
        return motion
    
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