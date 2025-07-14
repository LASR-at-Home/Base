#!/usr/bin/env python3
import rospy
from storing_groceries.state_machine import StoringGroceries

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon, PolygonStamped
from std_msgs.msg import Header

from shapely.geometry import Polygon as ShapelyPolygon
from shapely.validation import explain_validity


def check_polygons():

    table_publisher = rospy.Publisher(
        "/storing_groceries/table", PolygonStamped, queue_size=1, latch=True
    )

    table_polygon = ShapelyPolygon(rospy.get_param("/storing_groceries/table/polygon"))
    assert (
        table_polygon.is_valid
    ), f"Table polygon is not valid: {explain_validity(table_polygon)}"

    table_publisher.publish(
        PolygonStamped(
            polygon=Polygon(
                points=[
                    Point(x=x, y=y, z=0.0) for (x, y) in table_polygon.exterior.coords
                ]
            ),
            header=Header(frame_id="map"),
        )
    )

    shelf_publisher = rospy.Publisher(
        "/storing_groceries/shelves", PolygonStamped, queue_size=10, latch=True
    )

    for shelf in rospy.get_param("/storing_groceries/cabinet/shelves"):

        polygon = ShapelyPolygon(
            rospy.get_param(f"/storing_groceries/cabinet/shelves/{shelf}/polygon")
        )
        assert (
            polygon.is_valid
        ), f"{shelf['name']} polygon is not valid: {explain_validity(polygon)}"

        shelf_publisher.publish(
            PolygonStamped(
                polygon=Polygon(
                    points=[
                        Point(
                            x=x,
                            y=y,
                            z=rospy.get_param(
                                f"/storing_groceries/cabinet/shelves/{shelf}/z_min"
                            ),
                        )
                        for (x, y) in polygon.exterior.coords
                    ]
                ),
                header=Header(frame_id="map"),
            )
        )

    print("All polygons okay!")


if __name__ == "__main__":
    rospy.init_node("storing_groceries")
    check_polygons()
    use_arm = False
    sm = StoringGroceries(use_arm)
    sm.execute()
