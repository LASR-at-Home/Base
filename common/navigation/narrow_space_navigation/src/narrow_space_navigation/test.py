#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetMapResponse
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cv_bridge import CvBridge
import cv2

# Constants for the height map
SIZE = 50
IMPACT = 2
SPREAD = 8

def prob_density_function(x, mu, sigma):
    part1 = 1 / (sigma * math.sqrt(2 * math.pi)) # scaling of the curve
    part2 = math.pow(math.e, -0.5 * math.pow((x - mu) / sigma, 2))
    # sigma controls the spread of the curve
    # mu controls the center of the curve
    # part one controls the height
    return part1 * part2

def norm_dist():
    return np.random.normal(0, 1)

# def std(dist, impact, spread):
#     v1 = 1 / (impact * math.sqrt(2 * math.pi))
#     v2 = math.pow(dist, 2) / (2 * math.pow(spread, 2))
#     return v1 * math.pow(math.e, -v2)

def process_occupancy_grid(occupancy_grid_image):
    # Convert the Image message to a grayscale numpy array
    bridge = CvBridge()
    occupancy_array = bridge.imgmsg_to_cv2(occupancy_grid_image, desired_encoding='mono8')

    # Convert grayscale image to binary occupancy grid
    occupancy_array[occupancy_array < 128] = 0    # Black regions - Free space
    occupancy_array[occupancy_array >= 128] = 100 # White regions - Occupied

    # Initialize empty heightmap
    heights = np.zeros([SIZE, SIZE])

    # Process the occupancy grid and generate height map
    for x in range(0, occupancy_array.shape[1]):
        for y in range(0, occupancy_array.shape[0]):
            if occupancy_array[y][x] == 100:  # 100 means occupied
                continue
            for targetX in range(0, SIZE):
                if targetX >= 0 and targetX < SIZE:
                    for targetY in range(0, SIZE):
                        if targetY >= 0 and targetY < SIZE:
                            dist = math.sqrt((targetX - x) ** 2 + (targetY - y) ** 2)
                            heights[targetX][targetY] += std(dist, IMPACT, SPREAD)

    # do the same from the edges
    IMPACT = 4
    SPREAD = 6

    for x in range(0, SIZE):
        for y in range(0, SIZE):
            heights[x][y] += std(x, IMPACT, SPREAD) \
                + std(y, IMPACT, SPREAD) \
                + std(SIZE - x, IMPACT, SPREAD) \
                + std(SIZE - y, IMPACT, SPREAD)

    # Convert to points for visualization
    points = np.empty([SIZE * SIZE, 3])
    for x in range(0, SIZE):
        for y in range(0, SIZE):
            points[y * SIZE + x][0] = x
            points[y * SIZE + x][1] = y
            points[y * SIZE + x][2] = heights[x][y]

    # mark the points where people are standing for clarity
    standing = [
        (12, 25),
        (38, 17)
    ]
    for (x, y) in standing:
        heights[x][y] = 1

    # iterate through height map and find least busy point
    h = math.inf
    p = (0, 0)
    for x in range(0, SIZE):
        for y in range(0, SIZE):
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

    # Return the least busy point as a tuple (x, y)
    return p

def handle_get_height_map(req):
    # Process the occupancy grid image and get the least busy point
    least_busy_point = process_occupancy_grid(req)

    # Return the least busy point as the response
    return GetMapResponse(least_busy_point=least_busy_point)

def height_map_service():
    rospy.init_node('height_map_service')
    # rospy.Service('get_height_map', GetMap, handle_get_height_map)
    # rospy.spin()

if __name__ == "__main__":
    try:
        height_map_service()
    except rospy.ROSInterruptException:
        pass
