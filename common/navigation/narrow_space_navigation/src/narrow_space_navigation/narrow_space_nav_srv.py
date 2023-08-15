#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from narrow_space_navigation.srv import HeightMap, HeightMapResponse, HeightMapRequest
from nav_msgs.srv import GetMap, GetMapResponse
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cv_bridge import CvBridge
import cv2

# Constants for the height map


class NarrowSpaceNavSrv:
    def __init__(self):
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map = None

    def map_callback(self, msg):
        self.map = msg

    def get_map(self):
        return self.map

    def std(self, dist, impact, spread):
        v1 = 1 / (impact * math.sqrt(2 * math.pi))
        v2 = math.pow(dist, 2) / (2 * math.pow(spread, 2))
        return v1 * math.pow(math.e, -v2)

    def process_occupancy_grid(self, req, analytics):
        occupancy_grid_image = req
        plt.imshow(occupancy_grid_image, cmap='gray')
        plt.show()
        occupancy_array = occupancy_grid_image
        print(f" the list of unique vals {np.unique(occupancy_array.flatten())} ")
        print(f" the mean is {np.mean(occupancy_array.flatten())}")
        # Convert grayscale image to binary occupancy grid
        thresh = np.mean(occupancy_array.flatten())
        occupancy_array[occupancy_array < thresh] = 0  # Black regions - Free space
        occupancy_array[occupancy_array >= thresh] = 100  # White regions - Occupied
        plt.imshow(occupancy_array, cmap='gray')
        plt.show()
        # SIZE_Y = occupancy_array.shape[0]
        SIZE_Y = occupancy_array.shape[0]
        SIZE_X = SIZE_Y
        print(f" the size of the occupancy array is {occupancy_array.shape}")
        # SIZE_X = occupancy_array.shape[1]

        # Initialize empty heightmap
        heights = np.zeros([occupancy_array.shape[1], occupancy_array.shape[0]])

        IMPACT = 2
        SPREAD = 8

        # Process the occupancy grid and generate height map
        for x in range(0, SIZE_X):
            for y in range(0, SIZE_Y):
                if occupancy_array[y][x] == 100:  # 100 means occupied
                    # heights[int(x)][int(y)] = 1
                    continue
                for targetX in range(0, SIZE_X):
                    if targetX >= 0 and targetX < SIZE_X:
                        for targetY in range(0, SIZE_Y):
                            if targetY >= 0 and targetY < SIZE_Y:
                                dist = math.sqrt((targetX - x) ** 2 + (targetY - y) ** 2)
                                heights[targetX][targetY] += self.std(dist, IMPACT, SPREAD)

        # do the same from the edges
        IMPACT = 4
        SPREAD = 6

        for x in range(0, SIZE_X):
            for y in range(0, SIZE_Y):
                heights[x][y] += self.std(x, IMPACT, SPREAD) \
                                 + self.std(y, IMPACT, SPREAD) \
                                 + self.std(SIZE_X - x, IMPACT, SPREAD) \
                                 + self.std(SIZE_Y - y, IMPACT, SPREAD)

        # Convert to points for visualization
        points = np.empty([SIZE_X * SIZE_Y, 3])
        for x in range(0, SIZE_X):
            for y in range(0, SIZE_Y):
                points[y * SIZE_Y + x][0] = x
                points[y * SIZE_Y + x][1] = y
                points[y * SIZE_Y + x][2] = heights[x][y]

        # mark the points where people are standing for clarity
        standing = analytics[0]
        print(f"standing {standing}")
        for (x, y) in standing:
            print(f" x {x} y {y}")
            heights[int(x)][int(y)] = 1

        # iterate through height map and find least busy point
        h = math.inf
        p = (0, 0)
        for x in range(0, SIZE_X):
            for y in range(0, SIZE_Y):
                c = heights[x][y]
                if c < h:
                    h = c
                    p = (x, y)

        # Draw 3D plot using Matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Height')
        plt.show()

        # Assuming min_x, min_y are the coordinates of the minimum value in the heightmap
        px = p[0]
        py = p[1]
        print(f" the min value is {px} and the point is {py}")
        # px = (p[0] / SIZE) * occupancy_array.shape[1]
        # py = (p[1] / SIZE) * occupancy_array.shape[0]
        fig, ax = plt.subplots()
        ax.imshow(occupancy_grid_image, cmap='gray')
        ax.plot(px, py, 'ro')  # 'ro' stands for red circle marker
        # Set labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Image with Point')

        return (px, py)

    def calculate_height(self, x, y, occupancy_array, max_height=1.0):
        if occupancy_array[y][x] == 100:  # Occupied (obstacle)
            return max_height  # Assign the maximum height for free space
        else:
            obstacle_distance = self.distance_to_nearest_obstacle(x, y, occupancy_array)
            sigma = 0.1 * max(occupancy_array.shape)  # Control the spread of the Gaussian-like function
            height = np.exp(-0.5 * (obstacle_distance / sigma) ** 2)
            return height

    def distance_to_nearest_obstacle(self, x, y, occupancy_array):
        min_distance = float('inf')
        for i in range(occupancy_array.shape[0]):
            for j in range(occupancy_array.shape[1]):
                if occupancy_array[i][j] == 100:  # Obstacle
                    distance = math.sqrt((x - j) ** 2 + (y - i) ** 2)
                    min_distance = min(min_distance, distance)
        return min_distance

    def build_height_map(self, occupancy_array):
        SIZE_Y, SIZE_X = occupancy_array.shape
        heights = np.zeros_like(occupancy_array, dtype=float)

        for x in range(SIZE_X):
            for y in range(SIZE_Y):
                if occupancy_array[y][x] == 0:
                    height = self.calculate_height(x, y, occupancy_array)
                    heights[y][x] = height
                elif occupancy_array[y][x] == 100:  # Wall (occupied cell)
                    # Add height contribution from walls
                    IMPACT_WALLS = 4
                    SPREAD_WALLS = 6
                    height_contribution = (
                                                  1 / (IMPACT_WALLS * math.sqrt(2 * math.pi))
                                          ) * (
                                                  math.pow(x, 2) / (2 * math.pow(SPREAD_WALLS, 2))
                                                  + math.pow(y, 2) / (2 * math.pow(SPREAD_WALLS, 2))
                                                  + math.pow(SIZE_X - x, 2) / (2 * math.pow(SPREAD_WALLS, 2))
                                                  + math.pow(SIZE_Y - y, 2) / (2 * math.pow(SPREAD_WALLS, 2))
                                          )
                    heights[y][x] += height_contribution

        return heights

    def choose_target_point(self, occupancy_array):
        heights = self.build_height_map(occupancy_array)
        self.plot_height(heights)
        min_height = np.min(heights)
        target_indices = np.where(heights == min_height)
        target_x = target_indices[1][0]
        target_y = target_indices[0][0]
        return (target_x, target_y)

    def plot_height(self, heights):
        # Plot the height map
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot_surface(self.heights, cmap='terrain')
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Height')
        # plt.show()

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        x, y = np.meshgrid(np.arange(heights.shape[1]), np.arange(heights.shape[0]))
        ax.plot_surface(x, y, heights, cmap='viridis')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Height')
        plt.title('3D Height Map')
        plt.show()



if __name__ == "__main__":
    rospy.init_node("narrow_space_nav_srv", anonymous=True)
    server = NarrowSpaceNavSrv()
    srv = rospy.Service('/narrow_space_nav_srv', HeightMap, server.process_occupancy_grid)
    rospy.loginfo("String to location server initialised")
    rospy.spin()
    # try:
    #     points = None
    #     warped = None
    #     height_map_service(points, warped)
    # except rospy.ROSInterruptException:
    #     pass
