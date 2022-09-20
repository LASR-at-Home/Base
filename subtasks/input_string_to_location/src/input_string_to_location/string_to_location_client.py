#!/usr/bin/env python3

import rospy
from input_string_to_location.srv import StringToLocation, StringToLocationRequest
from std_msgs.msg import String

def client(location: String):
    rospy.wait_for_service('/string_to_location')
    try:
        req = StringToLocationRequest()
        req.location = location
        str_to_pos = rospy.ServiceProxy('string_to_location', StringToLocation)
        resp = str_to_pos(req)
        return resp.is_reached
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
if __name__ == '__main__':
    rospy.init_node("string_to_location_client", anonymous=True)
    try:
        while not rospy.is_shutdown():
            input = rospy.wait_for_message('/lasr_web_server/text_input', String)
            client(input)
    except rospy.ROSInterruptException:
        pass