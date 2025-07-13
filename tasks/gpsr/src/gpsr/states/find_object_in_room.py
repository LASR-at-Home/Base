import smach
import rospy

from typing import List, Union
from shapely.geometry import Polygon as ShapelyPolygon
from geometry_msgs.msg import Pose, Polygon

from gpsr.states import FindObjectAtLoc
from lasr_skills import GoToLocation


class FindObjectInRoom(smach.StateMachine):

    def __init__(
        self,
        object_name: str,
        location_waypoints: List[Pose],
        location_polygons: List[Union[Polygon, ShapelyPolygon]],
    ):
        """
        State machine to find an object in a room based on specified waypoints and polygons.

        Args:
            object_name (str): Name of the object to find.
            location_waypoints (List[]): List of waypoints to search for the object.
            location_polygons (List[ShapelyPolygon]): List of polygons defining the search areas.
        """
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["object_found"],
        )

        self._object_name = object_name
        self._location_waypoints = location_waypoints
        self._location_polygons = [
            (
                polygon
                if isinstance(polygon, ShapelyPolygon)
                else ShapelyPolygon([(point.x, point.y) for point in polygon.points])
            )
            for polygon in location_polygons
        ]

        with self:
            for i, (waypoint, polygon) in enumerate(
                zip(self._location_waypoints, self._location_polygons)
            ):
                smach.StateMachine.add(
                    f"GoToLocation_{i}",
                    GoToLocation(location=waypoint),
                    transitions={
                        "succeeded": f"FindObjectAtLoc_{i}",
                        "failed": f"GoToLocation_{i}",
                    },
                )
                smach.StateMachine.add(
                    f"FindObjectAtLoc_{i}",
                    FindObjectAtLoc(
                        object_name=self._object_name,
                        location_polygon=polygon,
                    ),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": (
                            f"GoToLocation_{i+1}"
                            if i + 1 < len(self._location_waypoints)
                            else "failed"
                        ),
                    },
                )
