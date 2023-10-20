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
        pass

    def calculate_height(self, x, y, occupancy_array, max_height=1.0):
        if occupancy_array[y][x] == 100:  # Occupied (obstacle)
            return max_height
        else:
            obstacle_distance = self.distance_to_nearest_obstacle(x, y, occupancy_array)
            sigma = 0.1 * max(occupancy_array.shape)  # Control the spread
            height = np.exp(-0.5 * (obstacle_distance / sigma) ** 2)
            return height

    def distance_to_nearest_obstacle(self, x, y, occupancy_array):
        min_distance = float('inf')
        for i in range(occupancy_array.shape[0]):
            for j in range(occupancy_array.shape[1]):
                if occupancy_array[i][j] == 100:
                    distance = math.sqrt((x - j) ** 2 + (y - i) ** 2)
                    min_distance = min(min_distance, distance)
        return min_distance

    def build_height_map(self, occupancy_array):
        print(f"occupancy_array: {occupancy_array.shape}")
        SIZE_Y, SIZE_X = occupancy_array.shape
        heights = np.zeros_like(occupancy_array, dtype=float)

        WALL_VALUE = 1
        heights[0, :] = WALL_VALUE
        heights[SIZE_Y - 1, :] = WALL_VALUE
        heights[:, 0] = WALL_VALUE
        heights[:, SIZE_X - 1] = WALL_VALUE

        EDGE_VALUE = 100
        # Set the edges of the occupancy_array to be walls (occupied)
        occupancy_array[0, :] = EDGE_VALUE
        occupancy_array[SIZE_Y - 1, :] = EDGE_VALUE
        occupancy_array[:, 0] = EDGE_VALUE
        occupancy_array[:, SIZE_X - 1] = EDGE_VALUE

        for x in range(SIZE_X):
            for y in range(SIZE_Y):
                if occupancy_array[y][x] == 0:
                    height = self.calculate_height(x, y, occupancy_array)
                    heights[y][x] = height
                elif occupancy_array[y][x] == 100:  # Wall /occupied cell
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
        print(f"heights: {heights}")
        non_zero_heights = heights[heights > 0]
        min_height = np.min(non_zero_heights)
        print(f"heights: {heights}")
        target_indices = np.where(heights == min_height)
        target_x = target_indices[1][0]
        target_y = target_indices[0][0]
        rospy.loginfo("Target point: ({}, {})".format(target_x, target_y))
        return (target_x, target_y)

    def plot_height(self, heights):
        rospy.loginfo("Plotting height map")
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        x, y = np.meshgrid(np.arange(heights.shape[1]), np.arange(heights.shape[0]))
        ax.plot_surface(x, y, heights, cmap='viridis')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Height')
        plt.title('3D Height Map')

        from lift.defaults import PLOT_SAVE, PLOT_SHOW, TEST, DEBUG_PATH
        if PLOT_SHOW:
            plt.show()
        if PLOT_SAVE:
            plt.savefig(DEBUG_PATH + "/heightmap" + str(TEST) + ".jpg")


if __name__ == "__main__":
    rospy.init_node("narrow_space_nav_srv", anonymous=True)
    server = NarrowSpaceNavSrv()
    srv = rospy.Service('/narrow_space_nav_srv', HeightMap, server.process_occupancy_grid)
    rospy.loginfo("String to location server initialised")
    rospy.spin()
