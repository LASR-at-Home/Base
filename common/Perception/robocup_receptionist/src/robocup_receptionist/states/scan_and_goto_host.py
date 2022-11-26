import rospy
import smach
from geometry_msgs.msg import Pose, Point, Quaternion
from robocup_receptionist.utils import activate_robot_navigation
from robocup_receptionist.dialogflow_receptionist.receptionist_conversation_API import ReceptionistAPI
from dialogflow_speech.utils import talk

from sensor_msgs.msg import PointCloud2

from face_detection.srv import FaceDetectionPCL

api = ReceptionistAPI()


class GoTo(smach.State):
    def __init__(self, base_controller):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['location', 'prev'], output_keys=['prev'])
        self.base_controller = base_controller
    
    def execute(self, userdata):
        # activate navigation on the robot
        activate_robot_navigation(True)
        api.speak('calling go to function to somewhere')
        self.goal = userdata.location

        # go to radius if previous state was scanning the room to detect the host
        if userdata.prev == 'Scan':
            # rospy.sleep(2.) # give it a 2 seconds for obstacle aware nav
            # success = self.base_controller.sync_to_radius(self.goal.position.x, self.goal.position.y, radius=2.0, tol=0.2)
            
            # if success:
            success = self.base_controller.sync_face_to(self.goal.position.x, self.goal.position.y)
        else:
            talk("I'm going to look for the host.")
            success = self.base_controller.sync_to_pose(self.goal)
        
        userdata.prev = 'GoTo'
        if success:
            rospy.loginfo(f"sending pose goal {self.goal.position} to base_controller...")
            return 'succeeded'
        else:
            rospy.loginfo(f"base controller failed to go to position {self.goal.position}")
            return 'failed'



# scan the living room
class Scan(smach.State):
    def __init__(self, head_controller):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['prev'], output_keys=['location', 'prev'])
        self.head_controller = head_controller
        rospy.wait_for_service("face_detection_pcl")
        self.face_detection = rospy.ServiceProxy("face_detection_pcl", FaceDetectionPCL)
    
    def execute(self, userdata):
        userdata.prev = 'Scan'
        # api.speak('looking for the host')
        def rotate_head(direction):
            self.head_controller.sync_reach_to(direction, 0.0, velocities=[0.1, 0.])

        # deactivate navigation on the robot to move the head around
        activate_robot_navigation(False)

        # run detection at most 4 times
        direction= 0.2
        host_name = rospy.get_param("host_name", "host")
        for _ in range(4):
            direction *= -1
            rotate_head(direction)
            print(f'rotating head in direction {direction}')
            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)

            resp = self.face_detection(pcl_msg)
            if resp.detections:
                for detection in resp.detections:
                    if detection.name == host_name:
                        rospy.loginfo('host position:')
                        print(detection.centroid.point)
                        talk("I found the host, follow me.")
                        userdata.location = Pose(detection.centroid.point, Quaternion(0, 0, 0, 1))
                        return 'succeeded'
        return 'failed'
 



class ScanAndGoToHostSM():
    def __init__(self, base_controller, head_controller):
        self.sm=smach.StateMachine(outcomes=['succeeded', 'failed'], input_keys=['guest_list'], output_keys=['guest_list'])

        if rospy.has_param('/room'):
            pose = rospy.get_param('/room')
        else:
            pose = {
                'position' : {
                    'x' : 2.91,
                    'y' : -2.12,
                    'z' : 0
                },
                'orientation' : {
                    'x' : 0,
                    'y' : 0,
                    'z' : 0.7,
                    'w' : 0.7
                }
            }
        self.sm.userdata.location = Pose(Point(pose['position']['x'],
                                               pose['position']['y'], 
                                               pose['position']['z']), 
                                        Quaternion(pose['orientation']['x'], 
                                                   pose['orientation']['y'], 
                                                   pose['orientation']['z'], 
                                                   pose['orientation']['w']))

        self.sm.userdata.prev = 'CollectInfoAndScan'

        self.base_controller = base_controller
        self.head_controller = head_controller
        
        with self.sm:
            self.sm.add('GO_TO_ROOM', GoTo(self.base_controller), 
                                    transitions={
                                                    'succeeded' : 'SCAN_ROOM', 
                                                    'failed'    : 'failed'
                                                },
                                    remapping={
                                                    'location': 'location',
                                                    'prev' : 'prev'
                                                })
            
            
            self.sm.add('SCAN_ROOM', Scan(self.head_controller), 
                                    transitions={
                                                    'succeeded' : 'GO_TO_HOST', 
                                                    'failed'    : 'failed'
                                                },
                                    remapping={
                                                    'location': 'location',
                                                    'prev' : 'prev'
                                                })

            
            self.sm.add('GO_TO_HOST', GoTo(self.base_controller), 
                                    transitions={
                                                    'succeeded' : 'succeeded', 
                                                    'failed'    : 'failed'
                                                },
                                    remapping={
                                                    'location': 'location',
                                                    'prev' : 'prev'
                                                })