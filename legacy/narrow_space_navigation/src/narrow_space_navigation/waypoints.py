#!/usr/bin/env python3
import copy
import itertools
from math import sqrt
from typing import List, Any
import black
import numpy as np
import rospy
import rosservice
from nav_msgs.msg import OccupancyGrid
import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from sklearn.cluster import DBSCAN, KMeans
from narrow_space_navigation.waypoint_helpers import *
import math
from std_msgs.msg import Empty
from tiago_controllers.controllers.controllers import Controllers
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG

np.set_printoptions(threshold=np.inf)

import matplotlib.pyplot as plt

# FIG, AXS = plt.subplots(1, 2)
# axs[0].imshow(template, cmap='gray')
# axs[1].imshow(roi, cmap='gray')

def draw_cnp(grid, cnp):
    plt.imshow(grid, cmap='gray')
    plt.title('CNP' + str(TEST))
    plt.axis('off')
    plt.plot(cnp[0], cnp[1], 'ro')
    # plt.savefig('/home/nicole/robocup/rexy/waypoints/waypoints' + str(TEST) + '-3.jpg', dpi=300,
    #             bbox_inches='tight')
    plt.show()
def plot_clusters_1(points=None, labels=None, dilation=None, centers=None, msg=None):
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
    # plt.imshow(dilation, cmap='gray')
    # AXS[0].imshow(dilation, cmap='gray')

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

    plt.show()
    # plt.savefig('/home/nicole/robocup/rexy/combination' + str(TEST) + '.png', dpi=300,
    #             bbox_inches='tight')
    # plt.imshow(dilation, cmap='gray')
    #
    # line_intersect_poly(centers, dilation)
    # #
    # inters = find_intersections(combinations, msg)
    #
    # for inter in inters:
    #     plt.plot([inter[0][0], inter[1][0]], [inter[0][1], inter[1][1]], 'k-', color='r')
    #
    # plt.savefig('/home/nicole/robocup/rexy/intersects' + str(TEST) + '.png', dpi=300,
    #             bbox_inches='tight')
    # plt.show()
class Waypoint:
    def __init__(self):
        self._msg = None
        self.edges = None
        # self.window_size = 100

        self.window_size = 100
        self.contours = None
        self.window = None
        self.eps = 5
        self.min_samples = 2
        self.num_ppl_gbl = 0
        self.num_ppl_lcl = 0
        self.erosion_iterations = 5
        self.dilation_iterations = 3
        self.resolution = None
        self.window_center = None
        self.base_controller = Controllers().base_controller

    def global_costmap_cb(self):
        """
            This function waits for a message on '/move_base/global_costmap/costmap' topic
            and returns it.
        """
        self._msg = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid)
        return self._msg

    def local_costmap_cb(self):
        """
            This function waits for a message on '/move_base/global_costmap/costmap' topic
            and returns it.
        """
        self._msg = rospy.wait_for_message('/move_base/local_costmap/costmap', OccupancyGrid)
        return self._msg

    def get_pose(self, _x: float, _y: float, msg: OccupancyGrid, isRobot: bool):
        if isRobot:
            robot_pose = self.current_pose()
            x, y = robot_pose.position.x, robot_pose.position.y
        else:
            x, y = _x, _y
        x, y = (x - msg.info.origin.position.x) / msg.info.resolution, \
               (y - msg.info.origin.position.y) / msg.info.resolution

        return x, y

    def np_grid(self, msg: OccupancyGrid):
        return np.array(msg.data).reshape(msg.info.height, msg.info.width)

    @staticmethod
    def clear_costmap():
        rospy.wait_for_service('/move_base/clear_costmaps')
        try:
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_global_costmap_window(self, isRobot=True, _x=None, _y=None):
        return self.create_window(msg=self.global_costmap_cb(), isRobot=isRobot, _x=_x, _y=_y)

    # def get_local_costmap_window(self, isRobot=True, _x=None, _y=None):
    #     return self.local_costmap_cb(msg=self.local_costmap_cb(), isRobot=isRobot, _x=_x, _y=_y)

    @staticmethod
    def current_pose():
        msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        pose = msg.pose.pose
        return pose

    def create_window(self, msg: OccupancyGrid = None, isRobot: bool = True, _x: float = None, _y: float = None):
        x, y = self.get_pose(_x, _y, msg, isRobot=isRobot)
        ind = y * msg.info.width + x
        x, y, ind = int(x), int(y), int(ind)
        w_x_min = max(0, int(x - self.window_size))
        w_x_max = min(msg.info.width, int(x + self.window_size))
        w_y_min = max(0, int(y - self.window_size))
        w_y_max = min(msg.info.height, int(y + self.window_size))
        window = np.array(msg.data).reshape(msg.info.height, msg.info.width)[w_y_min:w_y_max, w_x_min:w_x_max]
        # plt.imshow(window, cmap='gray')
        # plt.show()
        # plt.savefig('/home/nicole/robocup/rexy/window' + str(TEST) + '.png', dpi=300, bbox_inches='tight')
        return window

    @staticmethod
    def mask_polygon(window: np = None, poly_points: List[List[float]] = None, threshold: int = 60,
                     is_white_mask: bool = True):
        """
           Masks a polygon in an image using bitwise or and white mask or black mask with bitwise and.

           :param window: The image to mask.
           :param poly_points: The points of the polygon to mask.
           :param threshold: The threshold value for masking.
           :param is_white_mask: If true, the mask will be white, otherwise it will be black.

           :return: The masked image.
        """
        try:
            temp_window = np.array(window).astype(np.int32)
            new_window = np.where(temp_window <= threshold, 0, 255).astype(np.int32)
            if is_white_mask:
                mask = np.ones_like(new_window, dtype=np.int32) * 255
                cv2.fillPoly(mask, [np.array(poly_points)], 0)
                masked_window = cv2.bitwise_or(new_window, mask)
            else:
                mask = np.ones_like(new_window, dtype=np.int32) * 0
                cv2.fillPoly(mask, [np.array(poly_points)], 255)
                masked_window = cv2.bitwise_and(new_window, mask)
            # plt.imshow(masked_window, cmap='gray')
            #
            # plt.savefig('/home/nicole/robocup/rexy/mask' + str(TEST) + '.png', dpi=300,
            #             bbox_inches='tight')
            # plt.show()
            return masked_window
        except Exception as e:
            print(f"An error occurred while masking the polygon: {e}")

    def get_black_pxl_count(self, pixels, threshold=200):
        return np.sum(pixels <= threshold)

    def get_white_pxl_count(self, pixels, threshold=200):
        return np.sum(pixels > threshold)

    @staticmethod
    def get_white_pxl(pixels, threshold=60):
        return np.where(pixels > threshold)

    @staticmethod
    def get_black_pxl(pixels, threshold=60):
        return np.where(pixels <= threshold)

    def has_enough_free_space(self, window):


        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(window, connectivity=4, ltype=cv2.CV_32S)
        print(f"num_labels: {num_labels}, labels: {labels}, stats: {stats}")

        mask = np.zeros(window.shape, dtype=np.uint8)

        for i in range(num_labels):
            componentMask = (labels == i).astype(np.uint8) * 255
            mask = cv2.bitwise_or(mask, componentMask)
            plt.figure(figsize=(8, 8))
            plt.subplot(121)
            plt.imshow(window, cmap='gray')
            plt.title("Image")
            plt.axis("off")

            # Display the mask image
            plt.subplot(122)
            plt.imshow(mask, cmap='gray')
            plt.title("componentMask")
            plt.axis("off")

            plt.show()

        min_required_area = 1000  # Adjust this value as needed
        area = stats[0, cv2.CC_STAT_AREA]
        if area >= min_required_area:
            # Check if the polygon can fit within the connected component
            if self.can_fit_polygon_in_connected_component(labels == 0):
                print("Found a connected component that can fit the polygon")
                return True
        # for label in range(num_labels):  # Skip background label 0
        #     area = stats[label, cv2.CC_STAT_AREA]
        #     if area >= min_required_area:
        #         Check if the polygon can fit within the connected component
                # if self.can_fit_polygon_in_connected_component(labels == label):
                #     print("Found a connected component that can fit the polygon")
                #     return True

        return False

    def can_fit_polygon_in_connected_component(self, connected_component_mask):
        footprint = get_footprint(self._msg)
        polygon_points = np.array(footprint, dtype=np.int32)
        from shapely.geometry import Point, Polygon
        polygon = Polygon(polygon_points)

        for y in range(connected_component_mask.shape[0]):
            for x in range(connected_component_mask.shape[1]):
                if connected_component_mask[y, x] == 0:  # Check if it's a black pixel
                    # Check if the point (x, y) is inside the polygon
                    point = Point(x, y)
                    if not polygon.contains(point):
                        print("Point ({}, {}) is outside the polygon".format(x, y))
                        return False

        return True



    def is_narrow_space(self, points):
        """
        Checks if the robot can fit in a narrow space of elevator

        :param elevator: The elevator object

        :return: True if the robot can fit, False otherwise

        """
        # get the window
        self.global_costmap_cb()
        window = np.array(self._msg.data).astype(np.int32).reshape(self._msg.info.height, self._msg.info.width)
        elevator_sides = self.transform_points(points, self._msg)
        # mask a polygon area from the window
        black_pixels = self.mask_polygon(window=window, poly_points=elevator_sides, threshold=60)
        # get the number of black pixels
        pixels_count = self.get_black_pxl_count(black_pixels)
        # get robot footprint
        footprint = get_footprint(self._msg)
        # show_robot_footprint(footprint=footprint, window=window)
        # calculate if the robot fits
        return not is_robot_fit_in_elevator(window=window, footprint=footprint, pixels=pixels_count)

    @staticmethod
    def transform_points(points, msg):
        """
        Transforms the points to pixel coordinates

         :param points: The points to transform
         :param msg: The occupancy grid message

         :return: The transformed points
        """

        pxl_pts = []
        for pt in points:
            x, y = (pt[0] - msg.info.origin.position.x) / msg.info.resolution, \
                   (pt[1] - msg.info.origin.position.y) / msg.info.resolution
            x, y = int(x), int(y)
            pxl_pts.append([x, y])
        return pxl_pts

    def extract_given_elevator_warping(self, points, msg: OccupancyGrid):
        print("extract_given_elevator_warping")

        # Transform the points to pixel coordinates
        pxl_pts = self.transform_points(points, msg)
        pxl_pts = np.array(pxl_pts, dtype=np.float32)

        h_ab = np.sqrt(((pxl_pts[0][0] - pxl_pts[1][0]) ** 2) + ((pxl_pts[0][1] - pxl_pts[1][1]) ** 2))
        h_cd = np.sqrt(((pxl_pts[2][0] - pxl_pts[3][0]) ** 2) + ((pxl_pts[2][1] - pxl_pts[3][1]) ** 2))
        w_ad = np.sqrt(((pxl_pts[0][0] - pxl_pts[3][0]) ** 2) + ((pxl_pts[0][1] - pxl_pts[3][1]) ** 2))
        w_bc = np.sqrt(((pxl_pts[2][0] - pxl_pts[1][0]) ** 2) + ((pxl_pts[2][1] - pxl_pts[1][1]) ** 2))

        max_height = max(h_ab, h_cd).astype(int)
        max_width = max(w_bc, w_ad).astype(int)

        dest_pts = np.array([[0, 0],
                             [0, max_height - 1],
                             [max_width - 1, max_height - 1],
                             [max_width - 1, 0]], dtype=np.float32)

        # do the warp perspective
        M = cv2.getPerspectiveTransform(pxl_pts, dest_pts)
        grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        warped = cv2.warpPerspective(grid, M, (max_width, max_height), flags=cv2.INTER_NEAREST)


        # apply erosion
        warped = cv2.convertScaleAbs(warped)
        plt.imshow(warped, cmap='gray', aspect='auto')
        plt.axis('off')

        if PLOT_SHOW:
            plt.show()
        if PLOT_SAVE:
            plt.savefig(DEBUG_PATH + "/warped_in_extract" + str(TEST) + ".jpg")


        self.window = warped
        return warped, M

    def find_free_location_in_elevator(self, M=None, warped=None, elevator_number=None, is_cnp=False):
        # self.find_number_clusters(warped)
        centers, num_clusters, midpoints, dilation = self.find_clusters(warped)
        if num_clusters == 0:
            # return central pose
            return "central"
        elif not self.is_narrow_space():
            if is_cnp:
                cnps = self.find_multiple_cnp(dilation)
                global_cnp = self.local_to_global_points(M=M, points=cnps)
                points = global_cnp
            else:
                points = centers
            return points
        else:
            return None

    def find_midpoint(self, point1=None, point2=None):
        """
        This function finds the midpoint between two points.

        :param point1: The first point.
        :param point2: The second point.
        :return: The midpoint between the two points.
        """
        x1, y1 = point1
        x2, y2 = point2
        return [(x1 + x2) / 2, (y1 + y2) / 2]

    def find_midpoints(self, cluster_centers=None):
        """
        This function finds the midpoints between every combination of two points in a list of cluster centers.

         :param cluster_centers: The list of cluster centers.
         :return: The midpoints between every combination of two points in the list.
        """
        midpoints = []

        # Find midpoint between every combination of two points
        for pair in itertools.combinations(cluster_centers, 2):
            midpoint = self.find_midpoint(point1=pair[0], point2=pair[1])
            midpoints.append(midpoint)

        return np.array(midpoints)

    def local_to_global_points(self, M=None, points=None, is_lift=False):
        """
        Transforms local points to global points, given the transformation matrix.

        :param M: Transformation matrix.
        :param points: Local points.
        [[31.36451613  5.29032258]
         [30.13093525 59.23309353]]
        [[5.071246116948551, -3.6797133524745664], [5.052497872489603, -5.057776002586291]]


        :return: numpy.ndarray Global points.
        """
        new_points = []
        # Get the x and y coordinates of the first point
        # x, y = points.flatten()[0], points.flatten()[1]

        # Loop through all the points and add each point to the new_points list
        print(f'points: {points}')
        if is_lift:
            try:
                new_points.append([points[0], points[1]])
            except:
                new_points.append([points[0][0], points[0][1]])
        else:
            for i in range(points.shape[0]):
                print(points[i][0], points[i][1])
                new_points.append([points[i][0], points[i][1], 1])

        # Invert the transformation matrix
        _, IM = cv2.invert(M)

        # Apply the transformation matrix to each point
        global_points = []
        for i in new_points:
            global_point = np.float32([i[0], i[1]] + [1])
            x, y, z = np.dot(IM, global_point)
            global_points.append((x / z, y / z))

        # Get the x and y coordinates of each transformed point

        # plt.imshow(self.np_grid(self._msg), cmap='gray', aspect='auto')
        for i in range(len(global_points)):
            x, y = global_points[i][0], global_points[i][1]
            # plt.scatter(x, y, c='r')

        robot_points = []
        x, y = 0, 0

        # Loop through all the transformed points and calculate them based on the costmap origin and resolution
        for i in range(len(global_points)):
            _x, _y = global_points[i][0], global_points[i][1]

            # Calculate the x and y coordinates of each point
            x, y = (_x * self._msg.info.resolution) + self._msg.info.origin.position.x, \
                   (_y * self._msg.info.resolution) + self._msg.info.origin.position.y

            robot_points.append([x, y])

        print("robot points  {}".format(robot_points))
        # plt.title('local_global' + str(TEST) + '.jpg')
        #
        # if PLOT_SHOW:
        #     plt.show()
        # if PLOT_SAVE:
        #     plt.savefig(DEBUG_PATH + "/waypoints" + str(TEST) + ".jpg")

        return robot_points

    def find_clusters(self, warped):
        """
        This function finds clusters in an image.
         :param warped: The image to find clusters in.
         :return: The centers of the clusters and the number of clusters found.
        """
        # warped = np.where(warped <= 70, 0, 255).astype(np.uint8)
        warped = np.where(warped <= np.mean(warped), 0, 255).astype(np.uint8)
        # Create a kernel
        kernel = np.ones((5, 5), np.uint8)

        # Erode the image
        erosion = cv2.erode(warped, kernel, iterations=self.erosion_iterations)

        # Dilate the image
        dilation = cv2.dilate(erosion, kernel, iterations=self.dilation_iterations)

        # Display the dilation
        plt.imshow(dilation, cmap='gray', aspect='auto')
        plt.title('dilation')

        if PLOT_SHOW:
            plt.show()
        if PLOT_SAVE:
            plt.savefig(DEBUG_PATH + "/dilation" + str(TEST) + ".jpg")


        # Get non-black pixels
        non_black_pixels = self.get_white_pxl(pixels=dilation, threshold=55)

        # Stack points
        points = np.column_stack((non_black_pixels[1], non_black_pixels[0]))

        # Find cluster
        try:
            num_clusters, cluster_labels, db = self.dbscan(points=points)
        except ValueError:
            rospy.logerr('No clusters found')
            return 0, 0, 0, dilation


        # Find centers
        centers = self.find_center_clusters(db=db, points=points)

        midpoints = self.find_midpoints(cluster_centers=centers)
        # try:
        #     plot_clusters(points=points, labels=cluster_labels, dilation=dilation, centers=midpoints, msg=self._msg)
        #
        #     # Plot clusters
        #     plot_clusters(points=points, labels=cluster_labels, dilation=dilation, centers=centers, msg= self._msg)
        # except IndexError or NotImplementedError as e:
        #     rospy.logerr("the error is {}".format(e))

        return centers, num_clusters, midpoints, dilation

    def dbscan(self, points=None):
        """
        This function applies DBSCAN to get cluster centers.

         :param points: The points to apply DBSCAN on.

         :return: The number of clusters found, labels and db.
        """
        # Apply DBSCAN
        db = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = db.labels_
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        rospy.loginfo('Estimated number of clusters== people: %d' % num_clusters)
        return num_clusters, labels, db

    def find_center_clusters(self, db=None, points=None):
        """
        This function finds the center of clusters.

         :param db: The cluster labels.
         :param points: The points to find centers for.

         :return: The centers of the clusters.
       """
        centers = []

        # Find center of each cluster
        for label in set(db.labels_):
            if label != -1:
                mask = db.labels_ == label
                cluster_points = points[mask]
                center = np.mean(cluster_points, axis=0)
                centers.append(center)

        # self.centers = centers
        return np.array(centers)

    # assume it is called only around elevator
    def local_costmap_window(self, msg: OccupancyGrid, _x: float = None, _y: float = None):
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        window = grid[0:(3 * h // 4), w // 2:w]
        fig, ax = plt.subplots(1, 2)
        ax[0].imshow(grid)
        # plt.show()
        ax[1].imshow(window, cmap='gray')
        plt.title('local_costmap and window' + str(TEST))
        # plt.savefig('/home/nicole/robocup/rexy/waypoints/local_costmap' + str(TEST) + '-1.png', dpi=300,
        #             bbox_inches='tight')
        # plt.show()
        edges = self.find_edges(window)
        print('Edges: {}'.format(len(edges)))
        # draw_edges(edges)
        contours = self.find_contours(edges)
        print('SHAPEEE: {}'.format(window.shape))

        # Loop through each contour and check if it's above/below the threshold
        filtered_contours = []
        threshold_point = (72, 115)  # (x, y) coordinates
        for contour in contours:
            # calculate the center coordinates of the contour using its bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            center_x, center_y = x + w // 2, y + h // 2

            # check if the contour is above or below the threshold point
            if threshold_point[1] - 35 < center_y < threshold_point[1] + 35:  # contour is above the threshold point
                if threshold_point[0] - 25 < center_x < threshold_point[0] + 25:
                    filtered_contours.append(contour)

        print('Filtered Contours LEN: {}'.format(len(filtered_contours) // 2))
        draw_contours(filtered_contours)
        print('Contours: {}'.format(len(contours)))
        filtered_contours = self.filter_contours(filtered_contours, window)
        print('Filtered Contours LEN: {}'.format(len(filtered_contours) // 2))
        return window

    @staticmethod
    def find_edges(window):
        """
        This code takes a window as input and returns a list of edges.
        It applies the Canny edge detection algorithm to find the edges.

        :param window: a window of the costmap

        :return: a list of edges
        """
        if window is None:
            return None
        edges = cv2.Canny(window.astype(np.uint8), 50, 150)
        # for dilation of edges in an image, might give better results sometimes
        # kernel = np.ones((5, 5), np.uint8)
        # edges = cv2.dilate(edges, kernel, iterations=1)
        return edges

    @staticmethod
    def find_contours(edges):
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    @staticmethod
    def find_edges3(contours):
        """
        This code takes a list of contours as input and creates
         a list of edges from each contour. It iterates through
         each contour and for each point in the contour, it creates
         an edge that goes from that point to the next point in the contour.
         If it reaches the end of the contour, it wraps around to the beginning
         to create a closing edge. The resulting list of edges is then returned as a numpy array.
        """
        edges = []
        for contour in contours:
            for i in range(len(contour)):
                edge_start = contour[i][0]
                edge_end = contour[(i + 1) % len(contour)][0]
                edges.append([edge_start, edge_end])

        # visualize_edges3(np.array(edges))
        return np.array(edges)

    @staticmethod
    def min_distance_between_edges(edge1, edge2):
        min_distance = np.inf
        edge1_norm = edge1 / np.max(edge1)
        edge2_norm = edge2 / np.max(edge2)
        for i in range(len(edge1_norm)):
            for j in range(len(edge2_norm)):
                distance = np.linalg.norm(edge1_norm[i] - edge2_norm[j])
                if distance < min_distance:
                    min_distance = distance
        return min_distance

    @staticmethod
    def find_extreme_points(edge1, edge2):
        extreme_points = None
        min_distance = np.inf
        edge1_norm = edge1 / np.max(edge1)
        edge2_norm = edge2 / np.max(edge2)
        for i in range(len(edge1_norm)):
            for j in range(len(edge2_norm)):
                distance = np.linalg.norm(edge1_norm[i] - edge2_norm[j])
                if distance < min_distance:
                    min_distance = distance
                    extreme_points = (edge1[i], edge2[j])
        return extreme_points

    def find_closest_points(self, edges):
        closest_points = None
        min_distance = np.inf
        # print(f'Number of edges: {len(edges)}')
        for i, ei in enumerate(edges):
            for j, ej in enumerate(edges):
                if i != j:
                    distance = self.min_distance_between_edges(ei, ej)
                    if distance < min_distance:
                        min_distance = distance
                        closest_points = self.find_extreme_points(ei, ej)
                        print(f'Closest points: {closest_points}')
        return closest_points

    @staticmethod
    def filter_contours(contours, window):
        """
        Filter the contours by area and plot the contours if needed.

        :param contours: a list of contours
        :param window: a window of the costmap

        :return: a list of filtered contours
        """
        # Filter contours by area
        filtered_contours = []
        min_contour_length = 5
        for c in contours:
            contour = []
            for i in c:
                if 2 < i[0][0] < window.shape[0] - 2 and 2 < i[0][1] < window.shape[1] - 2:
                    contour.append(i[0])
            if len(contour) > min_contour_length:
                filtered_contours.append(contour)
        print('Filtered contours: {}'.format(len(filtered_contours)))
        with open('/home/nicole/robocup/rexy/waypoint-filtered.txt', 'a') as f:
            f.write(str(len(filtered_contours) // 2) + str(TEST) + '\n')
        # plot(filtered_contours, window)
        return filtered_contours

    def filter_contours_type(self, contours=None, window=None):
        """
        Filters contours based on their type.

        :param contours: List of contours.
        :type contours: list
        :param window: Image window.
        :type window: numpy.ndarray

        :return: List of filtered contours.
        """
        filtered_contours = []
        for c in contours:
            contour = []
            for i in c:
                if 2 < i[0][0] < window.shape[0] - 2 and 2 < i[0][1] < window.shape[1] - 2:
                    contour.append(i[0])
            filtered_contours.append(contour)

        print('Filtered contours: {}'.format(len(filtered_contours)))
        with open('/home/nicole/robocup/rexy/waypoint-filtered.txt', 'a') as f:
            f.write(str(len(filtered_contours) // 2) + str(TEST) + '\n')
        # plot(filtered_contours, self.window)
        return filtered_contours

    @staticmethod
    def min_dist_two_contours(contour1, contour2):
        # Use inefficient loop over all points!
        min_dist_point = None
        a_point = []  # Point in C1
        b_point = []  # Point in C2

        for i in contour1:
            for j in contour2:
                dist_point = sqrt(pow(j[0] - i[0], 2) + pow(j[1] - i[1], 2))
                if min_dist_point is None:
                    min_dist_point = dist_point
                    a_point = i
                    b_point = j
                elif dist_point < min_dist_point:
                    min_dist_point = dist_point
                    a_point = i
                    b_point = j

        # Point in the middle of the min distance line.
        try:
            final_point = a_point + (b_point - a_point) * 0.5
        except TypeError as e:
            rospy.logerr(e)
            final_point = None
        return min_dist_point, final_point

    def min_dist_contours(self, filtered_contours, window=None):
        """
        Select the two contours with the minimum distance between them.

        :param filtered_contours: a list of filtered contours

        :return: a list of selected contours
        """
        selected_contours = []
        min_d = 0.0
        Pcenter = []
        for c in filtered_contours:
            if len(selected_contours) == 0:
                # first contour
                selected_contours.append(c)
            elif len(selected_contours) == 1:
                # add contour and get min_dist
                selected_contours.append(c)
                min_d, Pcenter = self.min_dist_two_contours(selected_contours[0], selected_contours[1])
            else:
                # keep only the two contours with min distance between them
                min_d0, Pcenter0 = self.min_dist_two_contours(selected_contours[0], c)
                min_d1, Pcenter1 = self.min_dist_two_contours(selected_contours[1], c)

                if min_d0 < min_d1:
                    if min_d0 < min_d:
                        # keep only [0] and [c]
                        selected_contours.pop(1)
                        selected_contours.append(c)
                        Pcenter = Pcenter0
                else:
                    if min_d1 < min_d:
                        # keep only [1] and [c]
                        selected_contours.pop(0)
                        selected_contours.append(c)
                        Pcenter = Pcenter1
        # plot(selected_contours, self.window)
        return Pcenter

    def find_multiple_cnp(self, window, threshold=60):

        window = np.where(window <= threshold, 0, 255).astype(np.uint8)
        # plt.imshow(window, cmap='gray')
        # plt.title('dilation in find multiple cnp')
        # plt.show()
        # find edges within window
        edges = self.find_edges(window=window)

        # uncomment to visualise
        # draw_edges_on_window(window=window, edges=edges)

        # find contours within window
        contours = self.find_contours(edges=edges)
        # draw_contours(contours)

        # filter the contours in the correct type
        contours = self.filter_contours_type(contours=contours, window=self.window)
        print('Number of contours: {}'.format(len(contours)))
        with open('/home/nicole/robocup/rexy/waypoint-original.txt', 'a') as f:
            f.write(str("len contours ") + str(len(contours)) + str(TEST) + '\n')

        # create combinations of contours 1:1
        combinations = list(itertools.combinations(contours, 2))
        combinations = [list(elem) for elem in combinations]

        # all the critical points
        cnps = []

        for filtered_contours in combinations:
            pcenter = self.min_dist_contours(filtered_contours=filtered_contours)
            Dx = self._msg.info.origin.position.x
            Dy = self._msg.info.height * self._msg.info.resolution - abs(self._msg.info.origin.position.y)
            T = np.array([(1, 0, 0, Dx), (0, 1, 0, Dy), (0, 0, 1, 0), (0, 0, 0, 1)])
            cnps.append(pcenter)

        cnps = np.array([x for x in cnps if x is not None])

        for i in cnps:
            draw_cnp(window, i)
        return cnps

    def find_cnp(self):
        # define the window around the robot
        window = self.get_global_costmap_window()
        self._window = window
        # find edges within window
        edges = self.find_edges(window=window)
        # uncomment to visualise
        draw_edges_on_window(window=window, edges=edges)
        # find contours within window
        contours = self.find_contours(edges=edges)
        draw_contours(contours)
        print('Number of contours: {}'.format(len(contours)))
        # print('Number of contours: {}'.format(len(contours)))
        filtered_contours = self.filter_contours(contours=contours, window=window)
        pcenter = self.min_dist_contours(filtered_contours=filtered_contours)
        Dx = self._msg.info.origin.position.x
        Dy = self._msg.info.height * self._msg.info.resolution - abs(self._msg.info.origin.position.y)
        T = np.array([(1, 0, 0, Dx), (0, -1, 0, Dy), (0, 0, -1, 0), (0, 0, 0, 1)])

        # TODO: local to global coordinates
        # TODO: send to planner
        draw_cnp(self._window, pcenter)
        return [pcenter]

    def get_elevator_window(self, elevator_number=None):
        """
        Get the window of the elevator.

        :param window: a window of the costmap

        :return: a window of the elevator
        """
        # test lift
        # elevator_center = 3.66016769409, 0.0551229566336
        # points = [[2.12485527992, 1.20056855679],
        #           [2.4371213913, -0.712846040726],
        #           [4.71406841278, -0.426484644413],
        #           [4.14157915115, 1.20056855679]]
        # lift in the real world
        # elevator_center = 5.483719825744629, -4.394196510314941
        # points = [[3.23080015182, -1.33763432503],
        #           [4.57094621658, -1.10333895683],
        #           [5.05235815048, -2.76944160461],
        #           [3.45198941231, -3.251049757]]
        elevator_center = 5.353797912597656, -4.50060510635376
        points = [[4.282492637634277, -3.5537350177764893],
                  [4.353395938873291, -5.352860927581787],
                  [6.462785720825195, -5.264233589172363],
                  [6.569140911102295, -3.4651081562042236]]

        # uncomment to test with local costmap
        # self.local_costmap_cb()
        # self.local_costmap_cp_window(self._msg,_x=elevator_center[0],_y=elevator_center[1])
        # self.global_costmap_cb()
        self.get_global_costmap_window(isRobot=False,_x=elevator_center[0],_y=elevator_center[1])
        warped, M = self.extract_given_elevator_warping(points=points, msg=self._msg)
        print(warped)
        print(type(warped))
        l = self.find_free_location_in_elevator(M=M, warped=warped, elevator_number=elevator_number)
        # p = Pose()
        # p.position.x =  l[0][0]
        # p.position.y =  l[0][1]
        # p.orientation.w = 1
        # self.base_controller.sync_to_pose(p)

        # for i in range(100):
        #     self.get_global_costmap_window(isRobot=False,_x=elevator_center[0],_y=elevator_center[1])
        #     warped, M = self.extract_given_elevator_warping(points=points, msg=self._msg)


    def get_lift_information(self, is_lift, is_sim=True):
        # sim elevator
        if is_lift:
            c = rospy.get_param('lift_position_center_point')
            elevator_center = c[0], c[1]
            points = rospy.get_param('lift_position_points')
        else:
            c = rospy.get_param('wait_position_center_point')
            elevator_center = c[0], c[1]
            points = rospy.get_param('wait_position_points')

        if DEBUG > 3:
            print(f"center 1: {c}")
            print(f"center x: {c[0]} and the type {type(c[0])}")
            print(f"center y : {c[1]}")

        self.get_global_costmap_window(isRobot=False,_x=elevator_center[0],_y=elevator_center[1])
        warped, M = self.extract_given_elevator_warping(points=points, msg=self._msg)
        centers, num_clusters, midpoints, dilation = self.find_clusters(warped)
        analytics = [centers, num_clusters, midpoints, elevator_center]

        return warped, analytics, M



if __name__ == '__main__':
    rospy.init_node('waypoint_node')
    wp = Waypoint()
    # is narrow space working
    # print(wp.is_narrow_space())
    # find 1 cnp on doorways working + visualise
    # wp.find_cnp()
    # tested as well
    # start = rospy.Time.now().to_sec()
    wp.get_elevator_window()
    # end = rospy.Time.now().to_sec()
    # with open('/home/nicole/robocup/rexy/waypoints-timing.txt', 'a') as f:
    #     f.write(str(end - start) + '\n')
    # try:
    #     while not rospy.is_shutdown():
    #         rospy.spin()
    #         rospy.sleep(1)
    # except rospy.ROSInterruptException:
    #     pass
