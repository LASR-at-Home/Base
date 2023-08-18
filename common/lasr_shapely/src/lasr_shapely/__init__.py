import rospy

from lasr_shapely.srv import PointInPolygon2D

class LasrShapely:
    '''
    Wrapper class for Shapely service
    '''

    def __init__(self):
        self.point_proxy = rospy.ServiceProxy('/lasr_shapely/point_in_polygon_2d', PointInPolygon2D)
    
    def is_point_in_polygon_2d(self, polygon_2d_array, x, y):
        return self.point_proxy(polygon=[item for sublist in polygon_2d_array for item in sublist], x=x, y=y)
