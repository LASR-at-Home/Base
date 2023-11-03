#!/usr/bin/env python3
import rospy
import smach
from lasr_vision_msgs.srv import YoloDetection


class DetectPeople(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['img_msg'], output_keys=['people_detections'])
        self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

    def execute(self, userdata):
        try:
            result = self.yolo(userdata.img_msg, "yolov8n.pt", 0.5, 0.3)
            userdata.people_detections = result
            return 'succeeded'
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return 'failed'
        
if __name__ == "__main__":
    rospy.init_node("test_detect_people")
    sm = smach.StateMachine(outcomes=['end'])

    class MockA(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['1'], output_keys=['img_msg'])

        def execute(self, userdata):
            from sensor_msgs.msg import Image
            img_msg = rospy.wait_for_message("/usb_cam/image_raw", Image)
            userdata.img_msg = img_msg
            return '1' 

    class MockC(smach.State):

        def __init__(self):
            smach.State.__init__(self, outcomes=['1'], input_keys=['people_detections'])

        def execute(self, userdata):
            print(userdata.people_detections)
            return '1'

    with sm:
        sm.add('A', MockA(), transitions={'1' : 'DETECT_PEOPLE'})
        sm.add('DETECT_PEOPLE', DetectPeople(), transitions={'succeeded' : 'C', 'failed' : 'end'})
        sm.add('C', MockC(), transitions={'1' : 'A'})

    sm.execute()
