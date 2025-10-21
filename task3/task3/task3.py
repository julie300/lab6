import tkinter
import matplotlib #.pyplot as plt
matplotlib.use('TkAgg')

import numpy as np
import os
import matplotlib.pyplot as plt
from math import pi

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf_transformations import euler_from_quaternion


class lidar_subscriber(Node):
    def __init__(self):
        super().__init__('lidar_sbuscriber')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.update_plot)

        self.room_map = np.full((80, 80), -1)  # unknown
        self.room_map[40,40] = 1  # 로봇 시작 위치

        csv_data = np.genfromtxt('Task3_AgoraRoom.csv', delimiter=',')[100:820]
        angles = csv_data[:, 0] * np.pi / 180
        ranges = csv_data[:, 1]
        self.xs = ranges * np.cos(angles)
        self.ys = ranges * np.sin(angles)


        plt.ion()  # Turn on interactive mode for live updating plots

        fig = plt.figure(figsize=(12, 10))
        fig.suptitle("Lidar Mapping and Grid Representation", fontsize=16)

        # First, let's plot using polar coordinates the data in the file
        self.ax1 = fig.add_subplot(2, 2, 1, projection='polar')
        (self.polar_plot,) = self.ax1.plot([], [], '.')
        self.ax1.set_title("Polar Plot")

        # Plot cartesian view
        self.ax2 = fig.add_subplot(2, 2, 2)
        (self.cart_plot,) = self.ax2.plot([], [], '.')
        self.ax2.set_title("Cartesian Plot (x,y)")
        self.ax2.axis('equal')

        # Show the example map (as a matrix) with manually defining the cells' values
        self.ax3 = fig.add_subplot(2, 2, 3)
        self.ax3.imshow(self.room_map, cmap="gray")
        self.ax3.set_title("Sample Room Map before bresenham")

        # Show map with Bresenham points
        self.ax4 = fig.add_subplot(2, 2, 4)
        self.image = self.ax4.matshow(self.room_map, cmap='YlOrRd')
        #ax4.imshow(room_map, cmap="gray")
        self.ax4.set_title("Room Map with Bresenham Lines")

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()
    
    def odom_callback(self, msg):

        # Lidar position
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

        # Lidar orientation(yaw)
        self.quaternion = [msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w]
        self.yaw = euler_from_quaternion(self.quaternion)[2]  # roll, pitch, yaw

    def scan_callback(self, msg):
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        valid = np.isfinite(msg.ranges) & np.less(msg.range_min, msg.ranges) & np.less(msg.ranges, msg.range_max)
        ranges = np.array(msg.ranges)
        self.ranges = ranges[valid]
        self.angles = angles[valid]

        self.xs = self.ranges * np.cos(self.angles + self.yaw) + self.pose_x
        self.ys = self.ranges * np.sin(self.angles + self.yaw) + self.pose_y

        self.data = np.transpose(np.vstack((self.angles * 180 / pi, self.ranges)))
        self.grid_xs = (self.xs/0.05).astype(int)+40
        self.grid_ys = (self.ys/0.05).astype(int)+40

        h, w = self.room_map.shape
        for x, y in zip(self.grid_xs, self.grid_ys):
            if 0 <= x < w and 0 <= y < h:
                self.room_map[y, x] = 1 # occupied cell
                for px, py in self.bresenham_points([40, 40], [x, y]):
                    self.room_map[py, px] = 0 # free cell

        self.polar_plot.set_data(self.angles, self.ranges)
        self.cart_plot.set_data(self.xs, self.ys)
        self.ax2.relim()   
        self.ax2.autoscale_view()
        self.image.set_data(self.room_map)
    
    def bresenham_points(self, p0, p1):
        point_list = []  # We will fill this list with all points in between p0 and p1

        x0, y0 = p0[0], p0[1]
        x1, y1 = p1[0], p1[1]

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        if x0 < x1:      
            sx = 1
        else:            
            sx = -1

        if y0 < y1:      
            sy = 1
        else:            
            sy = -1

        err = dx - dy

        while True:
            if [x0, y0] != p0 and [x0, y0] != p1:  # exclude first and last
                point_list.append([x0, y0])
            if x0 == x1 and y0 == y1:
                break  # This means we have finished, so we break the loop

            e2 = 2 * err
            if e2 > -dy:
                # overshot in the y direction
                err = err - dy
                x0 = x0 + sx
            if e2 < dx:
                # overshot in the x direction
                err = err + dx
                y0 = y0 + sy

        return point_list
    
    def update_plot(self):
        self.image.set_data(self.room_map)
        plt.pause(0.001)



def main(args=None):
    rclpy.init(args=args)
    lidar_sub = lidar_subscriber()
    rclpy.spin(lidar_sub)
    lidar_sub.destroy_node()
    rclpy.shutdown()