from typing import Optional

import yasmin
from yasmin_ros import ServiceState

from lasr_vision_interfaces.srv import DetectGesture as DetectGestureSrv

from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon


class DetectGesture(ServiceState):
    """Query the gesture service for people making a gesture.

    Always uses 3D. If `area_polygon` is given, only locations inside it are
    kept, otherwise all are returned.
    """

    def __init__(
        self,
        gesture: str,
        area_polygon: Optional[ShapelyPolygon] = None,
        target_frame: str = "map",
        timeout: float = 5.0,
    ):
        super().__init__(
            srv_type=DetectGestureSrv,
            srv_name="/gesture/detect",
            create_request_handler=self._create_req,
            outcomes=["succeeded", "failed"],
            response_handler=self._response_handler,
        )
        self.set_description("Detects people making a gesture, returning 3D locations")
        self.add_output_key("gesture_locations")

        self.gesture = gesture
        self.area_polygon = area_polygon
        self.target_frame = target_frame
        self.timeout = timeout

    def _create_req(self, blackboard):
        return DetectGestureSrv.Request(
            gesture=self.gesture,
            use_3d=True,
            target_frame=self.target_frame,
            timeout=self.timeout,
        )

    def _response_handler(self, blackboard, response):
        blackboard["gesture_locations"] = []    # Empty previous gesture locations from memory
        locations = list(response.locations)

        if self.area_polygon is not None:
            if any(loc.header.frame_id != self.target_frame for loc in locations):
                yasmin.YASMIN_LOG_ERROR(
                    f"Gesture locations are not in '{self.target_frame}' (depth "
                    "unavailable?), cannot filter by polygon"
                )
                return "failed"
            locations = [
                loc
                for loc in locations
                if self.area_polygon.contains(ShapelyPoint(loc.point.x, loc.point.y))
            ]

        yasmin.YASMIN_LOG_INFO(
            f"Found {len(locations)} person(s) making '{self.gesture}'"
        )
        blackboard["gesture_locations"] = locations
        return "succeeded"
