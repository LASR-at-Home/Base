import rospy
import actionlib

from lasr_manipulation_msgs.msg import PlaceAction, PlaceGoal, PlaceResult


class PlaceServer:
    """
    An action server for placing objects.
    """

    def __init__(self) -> None:
        self._place_server = actionlib.SimpleActionServer(
            "/lasr_manipulation/place",
            PlaceAction,
            execute_cb=self._place,
            auto_start=False,
        )

        self._place_server.start()

        rospy.loginfo("/lasr_manipulation/place has started!")

    def _place(self, goal: PlaceGoal) -> None:
        pass


if __name__ == "__main__":
    rospy.init_node("lasr_manipulation_placing")
    place_server = PlaceServer()
    rospy.spin()
