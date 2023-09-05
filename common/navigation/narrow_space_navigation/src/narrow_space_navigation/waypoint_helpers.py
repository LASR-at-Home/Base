#!/usr/bin/env python3
import itertools

import matplotlib.pyplot as plt
import numpy as np
import rospy
# from skspatial.objects import Line
import cv2
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PolygonStamped, Polygon, Point32

np.set_printoptions(threshold=np.inf)

from lift.defaults import TEST

def draw_edges_on_window(window, edges):
    plt.imshow(window, cmap='gray')
    plt.show()
    fig, (ax1, ax2) = plt.subplots(1, 2)
    ax1.imshow(window, cmap='gray')
    ax1.set_title('Original Image')
    ax2.imshow(edges, cmap='gray')
    ax2.set_title('Edge Image')
    plt.savefig('/home/nicole/robocup/rexy/waypoints/waypoints' + str(TEST) + '-4.jpg', dpi=300,
                bbox_inches='tight')
    plt.show()


def draw_edges(edges):
    plt.imshow(edges, cmap='gray')
    plt.show()


def draw_contours(contours):
    for c in contours:
        plt.plot(c[:, 0, 0], c[:, 0, 1], 'r')
    plt.title('Contours ' + str(TEST))
    plt.savefig('/home/nicole/robocup/rexy/waypoints/waypoints-contours' + str(TEST) + '-5.jpg', dpi=300,
                bbox_inches='tight')

    plt.show()

import random
# def draw_cnp(grid, cnp):
#     plt.imshow(grid, cmap='gray')
#     plt.title('CNP' + str(TEST))
#     plt.plot(cnp[0], cnp[1], 'ro')
#     plt.show()


def draw_edge(edge, color='r'):
    count = 0
    for e in edge:
        plt.plot(e[:, 0], e[:, 1], color=color)
        count += 1
    plt.show()


def plot_centers(image, centers):
    # Plot original image
    plt.imshow(image, cmap='gray')
    plt.axis('off')
    # Iterate over cluster centers and plot a red circle at each center
    for center in centers:
        x, y = center.astype(int)
        plt.plot(x, y, 'ro', markersize=5)
    # Show image with centers
    plt.show()


def visualize_edges3(window, edges):
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']  # add more colors if needed
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(window, cmap='gray')
    ax.set_title('Edges')
    for i, edge in enumerate(edges):
        color = colors[i % len(colors)]
        ax.plot(edge[:, 0], edge[:, 1], '-', linewidth=2, color=color)
    plt.show()


def visualize_edges_test(window, edges):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(window, cmap='gray')
    ax.set_title('Edges')
    for edge in edges:
        ax.plot(edge[:, 0], edge[:, 1], '-', linewidth=2)
    plt.show()


def plot(contours, window):
    contours = [np.array(c) for c in contours]
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']  # add more colors if needed
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(window, cmap='gray')
    print(len(contours))
    for i, contour in enumerate(contours):
        color = colors[i % len(colors)]
        try:
            ax.plot(contour[:, 0], contour[:, 1], '+', linewidth=2, color=color)
        except IndexError as e:
            rospy.logerr(str(e) + str("changing the contour shape"))
            # contour = contour.reshape(-1, 2)
            # ax.plot(contour[:, 0], contour[:, 1], '+', linewidth=2, color=color)
    plt.title('Contours')
    plt.show()


def get_pose(_x, _y, msg, isRobot=True):
    if isRobot:
        pass
        # robot_pose = self.current_pose()
        # x, y = robot_pose.position.x, robot_pose.position.y
    else:
        x, y = _x, _y
    x, y = (x - msg.info.origin.position.x) / msg.info.resolution, \
           (y - msg.info.origin.position.y) / msg.info.resolution

    return x, y


def get_footprint(msg):
    footprint = rospy.wait_for_message('/move_base/global_costmap/footprint', PolygonStamped)
    footprint = footprint.polygon.points
    ft = []
    for p in footprint:
        p.x, p.y = get_pose(p.x, p.y, msg, isRobot=False)
        ft.append([p.x, p.y])
    return ft


def show_robot_footprint(window, footprint):
    new_points = np.array(footprint, dtype=np.int32)
    window = np.array(window, dtype=np.int32)
    mask = cv2.fillPoly(window, [new_points], 200)
    plt.imshow(mask, cmap='gray')
    plt.savefig('/home/nicole/robocup/rexy/footprint' + str(TEST) + '.jpg', dpi=300,
                bbox_inches='tight')
    plt.show()


def get_robot_pixels_in_window(window, footprint):
    new_points = np.array(footprint, dtype=np.int32)
    window = np.array(window, dtype=np.int32)
    mask = cv2.fillPoly(window, [new_points], 200)
    return np.sum(mask == 200)


def is_robot_fit_in_elevator(window, footprint, pixels):
    rospy.loginfo('showing fit in elevator for robot')
    robot_pixels = get_robot_pixels_in_window(window, footprint)
    return int(robot_pixels*2) < pixels


def area_with_shoelace(msg, elevator_points):
    footprint = get_footprint(msg)
    num_footprint_vertices = len(footprint)  # The number of vertices
    area = 0.0
    for i in range(num_footprint_vertices):
        j = (i + 1) % num_footprint_vertices  # The next vertex index (wrapping around)
        area += footprint[i].flatten()[0] * footprint[j].flatten()[1] - footprint[j].flatten()[0] * \
                footprint[i].flatten()[1]
        # The cross product of two adjacent vectors

    area /= 2.0  # Divide by two to get the signed area
    polygon_area = abs(area)  # Take absolute value to get positive area
    print("Area of polygon:", polygon_area)

    area_pixel_count_robot = polygon_area / (msg.info.resolution * msg.info.resolution)
    return area_pixel_count_robot * 3

from lift.defaults import PLOT_SAVE, PLOT_SHOW, TEST, DEBUG_PATH
def plot_clusters(points=None, labels=None, dilation=None, centers=None, msg=None):
    """
    This function plots clusters.

     :param points: The points to plot.
     :param labels: The cluster labels.
     :param dilation: The dilation image.
     :param centers: The centers of the clusters.
    """
    # Set up color map
    cmap = plt.get_cmap('jet')

    # Show dilation image
    plt.imshow(dilation, cmap='gray')

    # Set title and axis off
    plt.title('midpoints of clusters_' + str(TEST))

    plt.axis('off')

    # Plot points with color according to their cluster label
    for label in set(labels):
        if label != -1:
            mask = labels == label
            cluster_points = points[mask]
            color = cmap(label / len(set(labels)))
            plt.plot(cluster_points[:, 0], cluster_points[:, 1], '.', markersize=2, color=color)

    # Show image with clusters centers
    plt.scatter(centers[:, 0], centers[:, 1], c='r', s=10)

    combinations = list(itertools.combinations(centers, 2))
    combinations = [list(elem) for elem in combinations]
    #
    # for combination in combinations:
    #     plt.plot([combination[0][0], combination[1][0]], [combination[0][1], combination[1][1]], 'k-', color='r')

    if PLOT_SHOW:
        plt.show()
    if PLOT_SAVE:
        plt.savefig(DEBUG_PATH + "/clusters" + str(TEST) + ".jpg")

    # line_intersect_poly(centers, dilation)

    plt.imshow(dilation, cmap='gray')

    # inters = find_intersections(combinations, msg)
    #
    # for inter in inters:
    #     plt.plot([inter[0][0], inter[1][0]], [inter[0][1], inter[1][1]], 'k-', color='r')
    #
    if PLOT_SHOW:
        plt.show()
    if PLOT_SAVE:
        plt.savefig(DEBUG_PATH + "/dilation_in_plot_" + str(TEST) + ".jpg")

# from shapely.geometry import LineString, Polygon

def line_intersect_poly(centers, dilation):
    poly = Polygon(centers)
    lines = list(itertools.combinations(centers, 2))
    lines = [list(elem) for elem in lines]
    plt.imshow(dilation, cmap='gray')
    # for line in lines:
    #     plt.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], 'k-', color='r')
    intersections = []
    for line in lines:
        line = LineString(line)
        print(line)
        if line.intersects(poly):
            intersections.append(line.intersection(poly))
    plt.show()
    #
    # plt.imshow(dilation, cmap='gray')
    # for inter in intersections:
    #     plt.plot([inter[0][0], inter[1][0]], [inter[0][1], inter[1][1]], 'k-', color='r')
    # plt.show()
    filtered_lines = []
    for line in lines:
        x1, y1 = line[0]
        x2, y2 = line[1]

        # Check if the slope is different from any other line
        if all((x1 - x2) / (y1 - y2) != (x3 - x4) / (y3 - y4) for x3, y3 in centers for x4, y4 in centers if
               ((x3, y3) != (line[0][0], line[0][1]) and (x4, y4) != (line[1][0], line[1][0]))):
            filtered_lines.append(line)



    print(filtered_lines)
    plt.imshow(dilation, cmap='gray')
    for inter in filtered_lines:
        l , i = inter

        plt.plot([l[0][0], l[1][0]], [l[0][1], l[1][1]], 'k-', color='r')
        # plt.plot([inter[0][0], inter[1][0]], [inter[0][1], inter[1][1]], 'k-', color='r')
    plt.title('not working_' + str(TEST))
    plt.show()


def is_outside_window(width, height, x, y):
    if x < 0 or y < 0 or x >= width or y >= height:
        return True
    return False
def handle_outside_window(width, height, x, y):
    new_x = 0
    new_y = 0
    if x < 0:
        new_x = 0
    if y < 0:
        new_y = 0
    if x >= width:
        new_x = width
    if y >= height:
        new_y = height
    print('new_ points', new_x, new_y)
    return new_x, new_y
def find_intersections(combinations, msg):
    intersections_lines = []

    width, height = msg.info.width, msg.info.height

    tl = [0,0]
    tr = [width - 1,0]
    bl = [0,height - 1]
    br = [width - 1,height - 1]
    print('tl', tl, 'tr', tr, 'bl', bl, 'br', br)
    print(combinations, 'combinations')
    ab = Line.from_points(tl, tr)
    bc = Line.from_points(tr, br)
    cd = Line.from_points(br, bl)
    da = Line.from_points(bl, tl)
    sides = [ab, bc, cd, da]


    for combination in combinations:
        line1 = Line.from_points(combination[0], combination[1])
        intersections = []
        for side in sides:
            intersection_point = line1.intersect_line(side)
            if is_outside_window(width, height, intersection_point[0], intersection_point[1]):
                print('outside window', intersection_point)
                intersection_point = handle_outside_window(width, height, intersection_point[0], intersection_point[1])
            intersections.append(intersection_point)
        intersections_lines.append(intersections)

    return intersections_lines

if __name__ == '__main__':
    rospy.init_node('waypoint_node_helper')