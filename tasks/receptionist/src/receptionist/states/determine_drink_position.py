#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Point
from shapely.geometry import Polygon as ShapelyPolygon

class DetermineDrinkPosition(smach.State):
    """
    Splits your table into left/right halves and labels
    any point outside as 'centre'.
    """

    def __init__(self, table_area_param: str):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["drink_location"],
            output_keys=["drink_position_str"]
        )
        
        self._param = table_area_param #"/receptionist/table_area"

    def execute(self, ud):
        try:
            pts = rospy.get_param(self._param)
            bl, tl, tr, br = pts
        except Exception as e:
            rospy.logerr(f"Couldn’t read {self._param}: {e}")
            return "failed"

        #pt = ud.drink_location.point
        loc = ud.drink_location
        try:
            x, y = loc[0], loc[1]
        except (TypeError, IndexError):
            rospy.logerr("drink_location not in expected [x,y,w,h] format")
            return "failed"

        bottom_mid = Point((bl[0]+br[0])/2, (bl[1]+br[1])/2)
        top_mid    = Point((tl[0]+tr[0])/2, (tl[1]+tr[1])/2)

        left_poly = ShapelyPolygon([
            (bl[0],bl[1]),
            (bottom_mid.x,bottom_mid.y),
            (top_mid.x,top_mid.y),
            (tl[0],tl[1]),
        ])
        right_poly = ShapelyPolygon([
            (bottom_mid.x,bottom_mid.y),
            (br[0],br[1]),
            (tr[0],tr[1]),
            (top_mid.x,top_mid.y),
        ])
            
        if left_poly.contains((x, y)):
            ud.drink_position_str = "left"
        elif right_poly.contains((x, y)):
            ud.drink_position_str = "right"
        else:
            ud.drink_position_str = "centre"
        '''
        # with three polygons instead of 2
        bottom_left_mid = Point((bl[0]+br[0])/2, (bl[1]+br[1])/2)
        top_left_mid   = Point((tl[0]+tr[0])/2, (tl[1]+tr[1])/2)
        bottom_right_mid = Point((bl[0]+br[0])/2, (bl[1]+br[1])/2)
        top_right_mid    = Point((tl[0]+tr[0])/2, (tl[1]+tr[1])/2)

        left_poly = ShapelyPolygon([
            (bl[0],bl[1]),
            (bottom_left_mid.x,bottom_left_mid.y),
            (top_left_mid.x,top_left_mid.y),
            (tl[0],tl[1]),
        ])
        right_poly = ShapelyPolygon([
            (bottom_right_mid.x,bottom_right_mid.y),
            (br[0],br[1]),
            (tr[0],tr[1]),
            (top_right_mid.x,top_right_mid.y),
        ])

        centre_poly = ShapelyPolygon([
            (bottom_left_mid.x,bottom_left_mid.y),
            (bottom_right_mid.x,bottom_right_mid.y),
            (top_right_mid.x,top_right_mid.y),
            (top_left_mid.x,top_left_mid.y),
        ])
        
        if left_poly.contains((x, y)):
            ud.drink_position_str = "left"
        elif right_poly.contains((x, y)):
            ud.drink_position_str = "right"
        elif centre_poly.contains((x, y)):
            ud.drink_position_str =  "centre"
        else:
            return "failed"
        '''
        return "succeeded"
