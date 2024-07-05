#!/usr/bin/env python3
from go_to_location import GoToLocation
import smach
from lasr_skills import Detect3D,Detect3DInArea
from shapely.geometry.polygon import Polygon
from typing import List, Union, Dict
from geometry_msgs.msg import Pose, Point, Quaternion
from lasr_skills import Say, PlayMotion


"""
location = rospy.get_param("/start") -> python dict
Pose(position : Point, orientation : Quaternion)
pose = Pose(position=Point(**location['pose']['position'], orientation=Quaternion(**location['pose']['orientation']))
"""
class WaitForPersonInArea(smach.StateMachine):

    class CheckForPerson(smach.State):

        def __init__(self):
            smach.State.__init__(
                self, outcomes=["done", "not_done"], input_keys=["detections_3d"]
            )

        def execute(self, userdata):
            if len(userdata.detections_3d):
                return "done"
            else:
                return "not_done"

    def __init__(self, area_polygon: Polygon):
        smach.StateMachine.__init__(
            self,
            outcomes=["find_person", "not_find_person", "failed"],
            output_keys=["detections_3d"],
        )

        with self:
            smach.StateMachine.add(
                "DETECT_PEOPLE_3D",
                Detect3DInArea(area_polygon=area_polygon, filter=["person"]),
                transitions={"succeeded": "CHECK_FOR_PERSON", "failed": "failed"},
            )
            smach.StateMachine.add(
                "CHECK_FOR_PERSON",
                self.CheckForPerson(),
                transitions={"done": "find_person", "not_done": "not_find_person"},
            )


class Guiding_people(smach.StateMachine):
    class Detect_people(smach.State):
        # move head & detect people
        def __init__(self):
            smach.State.__init__(self, outcomes=['people_found','people_not_found','failed'], input_keys=['detections_3d'], output_keys=['detection_result'])
        def look_around(self, detections):
            print("we're checking on the follower")
            if len(detections) == 0:
                result = False
            else:
                result = True
            return result
            
        def execute(self, userdata):
            filtered_detections = userdata.detections_3d # the outcome of the 3d detection
            print(filtered_detections)
            object_list = filtered_detections
            result = self.look_around(object_list)
            userdata.detection_result = result
            rospy.loginfo(filtered_detections)
            if result:
                return 'people_found'
            else:
                return 'people_not_found'
    
    class cumulate_result(smach.State):
        def __init__(self): 
            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['detection_result','cumulated_result'], output_keys=['cumulated_result'])
        def execute(self, userdata):
            if 'cumulated_result' not in userdata:
                userdata.cumulated_result = list()
                userdata.cumulated_result.append(userdata.detection_result)
            else:
                userdata.cumulated_result.append(userdata.detection_result) # the outcome of the 3d detection
            return 'succeeded'
        
    class detection_result(smach.State):
        def __init__(self): 
            smach.State.__init__(self, outcomes=['check_found', 'check_not_found', 'failed'], input_keys=['cumulated_result'], output_keys=['result','cumulated_result'])

        def execute(self, userdata):
            if any(userdata.cumulated_result):
                userdata.result = True
                userdata.cumulated_result = list()
                return 'check_found'
            else:
                userdata.cumulated_result = list()
                userdata.result = False
                return 'check_not_found'
           

    def __init__(
        self,
        locations: Union[List[dict], None] = None,
        motions: Union[List[str], None] = None,
        waiting_area : Union[List[Polygon], None] = None,
        turning_point : Union[List[dict], None] = None,
  
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"]
        )
    

        with self:
            # 1. Start & greeting
            start_pose = Pose(position=Point(**locations[0]['pose']['position']), orientation=Quaternion(**locations[0]['pose']['orientation']))
            smach.StateMachine.add('START_GUIDING', GoToLocation(start_pose), transitions={'succeeded': "GREETING_GUEST_FIRST", 'aborted': 'failed', 'preempted': 'failed'})
            smach.StateMachine.add("GREETING_GUEST_FIRST", Say(text="I am waiting for a person."), transitions={"succeeded": "WAIT_FOR_PERSON_GUEST_1", "aborted": "failed", "preempted": "failed"})
            smach.StateMachine.add("WAIT_FOR_PERSON_GUEST_1", PlayMotion(motion_name=motions[0]), transitions={'succeeded': "GREETING_GUEST", 'aborted': "GREETING_GUEST", 'preempted': 'failed'})
            smach.StateMachine.add("GREETING_GUEST", Say(text="Please stand in front of me."), transitions={"succeeded": "DETECT_OBJECTS_3D_START", "aborted": "failed", "preempted": "failed"})

            # wait for person in area
            smach.StateMachine.add("DETECT_OBJECTS_3D_START", WaitForPersonInArea(waiting_area[0]),transitions={"find_person": "GREETING_GUEST_YES","not_find_person": "GREETING_GUEST_NO", "failed":"failed"})  
            smach.StateMachine.add("GREETING_GUEST_YES", Say(text="Hello, nice to meet you, please follow me."), transitions={"succeeded": "START_GUIDING_0", "aborted": "failed", "preempted": "failed"})
            smach.StateMachine.add("GREETING_GUEST_NO", Say(text="I didn't see anyone."), transitions={"succeeded": "GREETING_GUEST", "aborted": "failed", "preempted": "failed"})

            # 2. Go to locations
            for i, location in enumerate(locations[1:-1]):
                pose = Pose(position=Point(**location['pose']['position']), orientation=Quaternion(**location['pose']['orientation']))
                smach.StateMachine.add(f'START_GUIDING_{i}', GoToLocation(pose), transitions={'succeeded': f'TURNING_AROUND{i}', 'aborted': 'failed', 'preempted': 'failed'})
                
                turn_pose = Pose(position=Point(**turning_point[i]['pose']['position']), orientation=Quaternion(**turning_point[i]['pose']['orientation']))
                smach.StateMachine.add(f'TURNING_AROUND{i}', GoToLocation(turn_pose), transitions={'succeeded': f'CHECK_AROUND_{i}_0', 'aborted': 'failed', 'preempted': 'failed'})

                state = 0
                for index, motion in enumerate(motions):
                    smach.StateMachine.add(f'CHECK_AROUND_{i}_{state}', PlayMotion(motion_name=motion), transitions={'succeeded': f"DETECT_OBJECTS_{i}_{state}", 'aborted': f"DETECT_OBJECTS_{i}_{state}", 'preempted': 'failed'})
                    smach.StateMachine.add(f"DETECT_OBJECTS_{i}_{state}", WaitForPersonInArea(waiting_area[i+1]),transitions={"find_person": f'RESULT_{i}_{state}',"not_find_person": f'RESULT_{i}_{state}', "failed":"failed"})  
                    smach.StateMachine.add(f'RESULT_{i}_{state}', self.Detect_people(), transitions={'people_found': f'SAVE_RESULT_{i}_{state}', 'people_not_found': f'SAVE_RESULT_{i}_{state}', 'failed': 'failed'})
                    smach.StateMachine.add(f'SAVE_RESULT_{i}_{state}', self.cumulate_result(), transitions={'succeeded': f'CHECK_AROUND_{i}_{state+1}' if index != (len(motions) - 1) else f'CHECK_RESULT_{i}', 'failed': 'failed'})
                    state += 1

                smach.StateMachine.add(f'CHECK_RESULT_{i}', self.detection_result(), transitions={'check_found': f"GREETING_GUEST_YES_{i}", 'check_not_found': f"GREETING_GUEST_NO_{i}", 'failed': 'failed'})
                smach.StateMachine.add(f"GREETING_GUEST_YES_{i}", Say(text="Let's move on to the destination."), transitions={"succeeded": f'START_GUIDING_{i+1}' if index != (len(locations[1:-1])) else 'GUIDING_DESTINATION', "aborted": 'failed', 'preempted': 'failed'})
                smach.StateMachine.add(f"GREETING_GUEST_NO_{i}", Say(text="Hey, where are you? please follow me."), transitions={"succeeded": f'CHECK_AROUND_{i}_{state-2}', 'aborted': 'failed', 'preempted': 'failed'})

            # 3. Final destination
            end_pose = Pose(position=Point(**locations[-1]['pose']['position']), orientation=Quaternion(**locations[-1]['pose']['orientation']))
            smach.StateMachine.add(f'GUIDING_DESTINATION', GoToLocation(end_pose), transitions={'succeeded': "TURN_BACK", 'aborted': 'failed', 'preempted': 'failed'})
            end_turn_pose = Pose(position=Point(**turning_point[-1]['pose']['position']), orientation=Quaternion(**turning_point[-1]['pose']['orientation']))
            smach.StateMachine.add(f'TURN_BACK', GoToLocation(end_turn_pose), transitions={'succeeded': "SAY_ARRIVED", 'aborted': 'failed', 'preempted': 'failed'})
            smach.StateMachine.add("SAY_ARRIVED", Say(text="We've arrived at the destination. See you."), transitions={"succeeded": "succeeded", 'aborted': 'failed', 'preempted': 'failed'})


                

if __name__ == "__main__": 
    import rospy
    from sensor_msgs.msg import PointCloud2
    location = [rospy.get_param('/Start'), rospy.get_param("/WaitPoint1"), rospy.get_param("/WaitPoint2"), rospy.get_param(f"/Destination")]
    turn_around = [rospy.get_param(f"/TurnAround1"), rospy.get_param(f"/TurnAround2"), rospy.get_param('/TurnAroundDestination')]
    motion = ["look_center","look_left", "look_right"]
    point_1 = Polygon([[2.72,7.01],[5.09,7.57],[5.14, 6.13],[3.64,5.51]])
    point_2 = Polygon([[1.35,5.15],[2.47,5.4],[2.78, 4.25],[1.69,3.96]])
    point_3 = Polygon([[1.11,1.09],[3.36,1.73],[3.73, -0.03],[1.53,-0.60]])
    point = [point_1, point_2, point_3]
    rospy.init_node("guiding_people")
    sm = Guiding_people(locations=location, motions=motion, turning_point= turn_around, waiting_area= point)
    sm.userdata.pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    sm.execute()

    #wait for person in area


