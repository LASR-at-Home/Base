import rospy

import smach
import smach_ros

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyRequest

from lasr_manipulation_msgs.srv import AddSupportSurface, AddSupportSurfaceRequest


class SetupPlanningScene(smach.StateMachine):

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])

        with self:

            smach.StateMachine.add(
                "CLEAR_PLANNING_SCENE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/clear",
                    Empty,
                    request=EmptyRequest(),
                ),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

    #         smach.StateMachine.add(
    #             "ADD_TABLE_SURFACE",
    #             smach_ros.ServiceState(
    #                 "/lasr_manipulation_planning_scene/add_support_surface",
    #                 AddSupportSurface,
    #                 request=AddSupportSurfaceRequest(
    #                     "table",
    #                     PoseStamped(
    #                         header=Header(frame_id="map"),
    #                         pose=Pose(
    #                             position=Point(
    #                                 **rospy.get_param(
    #                                     "/storing_groceries/table/surface/pose/position"
    #                                 ),
    #                             ),
    #                             orientation=Quaternion(
    #                                 **rospy.get_param(
    #                                     "/storing_groceries/table/surface/pose/orientation"
    #                                 )
    #                             ),
    #                         ),
    #                     ),
    #                     Vector3(
    #                         **rospy.get_param(
    #                             "/storing_groceries/table/surface/dimensions"
    #                         )
    #                     ),
    #                 ),
    #                 output_keys=["success"],
    #                 response_slots=["success"],
    #             ),
    #             transitions={
    #                 "succeeded": "ADD_SHELF_SURFACES",
    #                 "preempted": "failed",
    #                 "aborted": "failed",
    #             },
    #         )

    #         shelf_iterator = smach.Iterator(
    #             outcomes=["succeeded", "failed"],
    #             it=rospy.get_param("/storing_groceries/cabinet/shelves"),
    #             it_label="shelf_name",
    #             exhausted_outcome="succeeded",
    #             input_keys=[],
    #             output_keys=[],
    #         )

    #         with shelf_iterator:
    #             container_sm = smach.StateMachine(
    #                 outcomes=["continue", "succeeded", "failed"],
    #                 input_keys=["shelf_name"],
    #             )

    #             with container_sm:
    #                 smach.StateMachine.add(
    #                     "ADD_SUPPORT_SURFACE",
    #                     smach_ros.ServiceState(
    #                         "/lasr_manipulation_planning_scene/add_support_surface",
    #                         AddSupportSurface,
    #                         request_cb=self._shelf_request_cb,
    #                         output_keys=["success"],
    #                         response_slots=["success"],
    #                         input_keys=["shelf_name"],
    #                     ),
    #                     transitions={
    #                         "succeeded": "continue",
    #                         "preempted": "failed",
    #                         "aborted": "failed",
    #                     },
    #                 )
    #             smach.Iterator.set_contained_state(
    #                 "CONTAINER_SM", container_sm, loop_outcomes=["continue"]
    #             )

    #         smach.StateMachine.add(
    #             "ADD_SHELF_SURFACES",
    #             shelf_iterator,
    #             transitions={"succeeded": "succeeded", "failed": "failed"},
    #         )

    # def _shelf_request_cb(self, userdata, _):
    #     return AddSupportSurfaceRequest(
    #         userdata.shelf_name,
    #         PoseStamped(
    #             header=Header(frame_id="map"),
    #             pose=Pose(
    #                 position=Point(
    #                     **rospy.get_param(
    #                         f"/storing_groceries/cabinet/shelves/{userdata.shelf_name}/surface/pose/position"
    #                     ),
    #                 ),
    #                 orientation=Quaternion(
    #                     **rospy.get_param(
    #                         f"/storing_groceries/cabinet/shelves/{userdata.shelf_name}/surface/pose/orientation"
    #                     )
    #                 ),
    #             ),
    #         ),
    #         Vector3(
    #             **rospy.get_param(
    #                 f"/storing_groceries/cabinet/shelves/{userdata.shelf_name}/surface/dimensions"
    #             )
    #         ),
    #     )
