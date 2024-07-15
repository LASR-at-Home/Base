#!/usr/bin/env python3
import rospy

from lasr_vision_grounded_sam.grounded_sam import run_grounded_sam, load_model
from lasr_vision_msgs.srv import GroundedSamRequest, GroundedSamResponse, GroundedSam

MODEL = load_model()


def grounded_sam_request(req: GroundedSamRequest) -> GroundedSamResponse:
    rospy.loginfo("Received Grounded Sam Request, dispatching...")
    response: GroundedSamResponse = run_grounded_sam(req, MODEL)
    rospy.loginfo("Grounded Sam request processed")
    return response


rospy.init_node("grounded_same_service")
rospy.Service("/vision/grounded_sam", GroundedSam, grounded_sam_request)
rospy.loginfo("Grounded Sam service started")
rospy.spin()
