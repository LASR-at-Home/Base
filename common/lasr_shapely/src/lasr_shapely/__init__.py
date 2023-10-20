import rospy

from lasr_shapely.srv import PointInPolygon2D, PointsInPolygon2D

class LasrShapely:
    '''
    Wrapper class for Shapely service
    '''

    def __init__(self):
        rospy.wait_for_service('/lasr_shapely/point_in_polygon_2d')

        self.point_proxy = rospy.ServiceProxy('/lasr_shapely/point_in_polygon_2d', PointInPolygon2D)
        self.points_proxy = rospy.ServiceProxy('/lasr_shapely/points_in_polygon_2d', PointsInPolygon2D)
    
    def is_point_in_polygon_2d(self, polygon_2d_array, x, y):
        return self.point_proxy(polygon=[item for sublist in polygon_2d_array for item in sublist], x=x, y=y)
    
    def are_points_in_polygon_2d(self, polygon_2d_array, points_2d_array):
        return self.points_proxy(polygon=[item for sublist in polygon_2d_array for item in sublist],
                                points=[item for sublist in points_2d_array for item in sublist])
    
    def are_points_in_polygon_2d_flatarr(self, polygon_2d_array, points_2d_array):
        return self.points_proxy(polygon=polygon_2d_array,
                                points=points_2d_array)

if __name__=='__main__':
    rospy.init_node('testgkfdp', anonymous=True)
    print(LasrShapely().is_point_in_polygon_2d([[0, 0], [5,0], [10, 10], [0, 5]], 6, 6))
    print(LasrShapely().are_points_in_polygon_2d([[0, 0], [5,0], [10, 10], [0, 5]], [[1,2], [11, 11]]))
