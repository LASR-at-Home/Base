import smach
from .states import (
    GoToWaitLocation,
    LookForPerson,
    GoToPerson,
    GreetPerson,
    GuidePerson,
    Start,
    LookForPersonLaser,
    GoCloserToPerson,
)
import rospy
from shapely.geometry import Polygon
from lasr_skills import WaitForPersonInArea

from geometry_msgs.msg import PointStamped, Point
class Phase3(smach.StateMachine):
    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:
            smach.StateMachine.add(
                "START_PHASE_3",
                Start(context),
                transitions={"done": "GO_TO_WAIT_LOCATION"},
            )
            smach.StateMachine.add(
                "GO_TO_WAIT_LOCATION",
                GoToWaitLocation(context),
                transitions={"done": "done", "not done": "LOOK_FOR_PERSON_LASER"},
            )

            
            smach.StateMachine.add(
                "LOOK_FOR_PERSON_LASER",
                LookForPersonLaser(context),
                transitions={
                    "found": "GO_CLOSER_TO_PERSON",
                    "not found": "LOOK_FOR_PERSON_LASER",
                },
            )
            '''
            corners = rospy.get_param("/coffee_shop/wait/cuboid")
            wait_area = Polygon(corners)

            smach.StateMachine.add(
            "LOOK_FOR_PERSON_LASER",
            WaitForPersonInArea(wait_area),   
            transitions={"succeeded": "GO_CLOSER_TO_PERSON", "failed": "LOOK_FOR_PERSON_LASER"}
            ,
            
            remapping={"detections_3d": "person_detections"},
            )

            '''
            '''
            @smach.cb_interface(input_keys=["person_detections"], outcomes=["done"])
            def set_person_target_cb(ud):
                if not ud.person_detections:
                    return "done"  

                det = ud.person_detections[0]
                # adapt to your Detect3DInArea output
                if hasattr(det, "position"):
                    x, y, z = det.position.x, det.position.y, det.position.z
                else:
                    x = det.pose.pose.position.x
                    y = det.pose.pose.position.y
                    z = det.pose.pose.position.z

                ps = PointStamped()
                ps.header.frame_id = "map"  
                ps.point = Point(x, y, z)

                context.new_customer_pose = ps
                if hasattr(context, "publish_person_pose"):
                    context.publish_person_pose(x, y, z, ps.header.frame_id)
                return "done"

            smach.StateMachine.add(
                "SET_PERSON_TARGET",
                smach.CBState(set_person_target_cb),
                transitions={"done": "GO_CLOSER_TO_PERSON"},
                remapping={"person_detections": "person_detections"},
            )

            '''
            smach.StateMachine.add(
                "GO_CLOSER_TO_PERSON",
                GoCloserToPerson(context),
                transitions={"done": "LOOK_FOR_PERSON"},
            )
            smach.StateMachine.add(
                "LOOK_FOR_PERSON",
                LookForPerson(context),
                transitions={"found": "GO_TO_PERSON", "not found": "LOOK_FOR_PERSON"},
            )
            smach.StateMachine.add(
                "GO_TO_PERSON",
                GoToPerson(context),
                transitions={"done": "GREET_PERSON"},
            )
            smach.StateMachine.add(
                "GREET_PERSON",
                GreetPerson(context),
                transitions={"done": "GUIDE_PERSON"},
            )
            smach.StateMachine.add(
                "GUIDE_PERSON",
                GuidePerson(context),
                transitions={"done": "done"},
            )
