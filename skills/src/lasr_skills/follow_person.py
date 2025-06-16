#!/usr/bin/env python3
import rospy
import smach
from lasr_person_following.srv import FollowPersonSrv, FollowPersonSrvRequest
from typing import Union, Dict, Any


class FollowPersonState(smach.State):
    """
    A skill that makes the robot follow a person using the person_following_service.

    Parameters:
        max_following_distance (float): Maximum distance to follow before asking person to wait (meters)
        max_speed (float): Maximum robot velocity (m/s)
        stopping_distance (float): Distance to maintain from person (meters)
        speak (bool): Whether robot should speak during following
        timeout (float): Maximum following time (seconds), 0 for no timeout

    Outcomes:
        succeeded: Person following completed successfully
        lost_person: The person was lost during following
        timed_out: Following timed out (if timeout > 0)
        failed: Failed to start following or service call failed

    Output keys:
        distance_traveled (float): Total distance traveled during following
        following_duration (float): Total time spent following (seconds)
        result_type (str): Reason for finishing (ARRIVED, LOST_PERSON, TIMEOUT, etc.)
    """

    def __init__(
        self,
        max_following_distance: float = 3.5,
        max_speed: float = 0.4,
        stopping_distance: float = 2.0,
        speak: bool = True,
        timeout: float = 0.0,
    ):
        # Initialize SMACH state
        smach.State.__init__(
            self,
            outcomes=["succeeded", "lost_person", "timed_out", "failed"],
            output_keys=["distance_traveled", "following_duration", "result_type"],
        )

        # Store configuration parameters
        self._max_following_distance = max_following_distance
        self._max_speed = max_speed
        self._stopping_distance = stopping_distance
        self._speak = speak
        self._timeout = timeout

        # Service name for person following
        self._service_name = "/follow_person"

    def execute(self, userdata: Dict[str, Any]) -> str:
        """
        Execute the person following skill.

        Args:
            userdata: User data passed from the state machine

        Returns:
            str: Outcome of the skill execution
        """
        try:
            # Wait for the service to be available
            rospy.loginfo(f"Waiting for '{self._service_name}' service...")
            rospy.wait_for_service(self._service_name, timeout=5.0)

            # Create service proxy
            follow_person_srv = rospy.ServiceProxy(self._service_name, FollowPersonSrv)

            # Create service request
            request = FollowPersonSrvRequest()
            request.max_following_distance = self._max_following_distance
            request.max_speed = self._max_speed
            request.stopping_distance = self._stopping_distance
            request.speak = self._speak
            request.timeout = self._timeout

            # Call the service
            rospy.loginfo("Starting person following...")
            response = follow_person_srv(request)

            # Store results in userdata
            userdata.distance_traveled = response.distance_traveled
            userdata.following_duration = response.following_duration
            userdata.result_type = response.result_type

            # Determine outcome based on result_type
            if not response.success:
                rospy.logwarn(f"Person following failed: {response.result_type}")
                return "failed"

            if response.result_type == "LOST_PERSON":
                rospy.loginfo("Person was lost during following")
                return "lost_person"

            if response.result_type == "TIMEOUT":
                rospy.loginfo("Person following timed out")
                return "timed_out"

            # Default successful outcome
            # result_type will be ARRIVED
            rospy.loginfo(
                f"Person following completed successfully: {response.result_type}"
            )
            return "succeeded"

        except rospy.ROSException as e:
            rospy.logerr(f"Service timeout: {e}")
            return "failed"
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return "failed"
        except Exception as e:
            rospy.logerr(f"Unexpected error during person following: {e}")
            return "failed"


# Example usage in a larger state machine
class FollowPerson(smach.StateMachine):
    def __init__(
        self,
        max_following_distance: float = 3.5,
        max_speed: float = 0.4,
        stopping_distance: float = 2.0,
        speak: bool = True,
        timeout: float = 0.0,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
        )

        with self:
            # Add the FollowPersonState skill
            smach.StateMachine.add(
                "FOLLOW_PERSON",
                FollowPersonState(
                    max_following_distance=max_following_distance,
                    max_speed=max_speed,
                    stopping_distance=stopping_distance,
                    speak=speak,
                    timeout=timeout,
                ),
                transitions={
                    "succeeded": "succeeded",
                    "lost_person": "SAY_LOST",
                    "timed_out": "SAY_TIMEOUT",
                    "failed": "failed",
                },
            )

            # Add other states like Say for different outcomes
            # (These would be imported from lasr_skills in a real implementation)
            # For example: smach.StateMachine.add('SAY_ARRIVED', Say(text="We have arrived!"), ...)


# For testing the skill individually
if __name__ == "__main__":
    rospy.init_node("follow_person_skill_test")

    # Create a simple state machine with just the FollowPersonState skill
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])

    with sm:
        smach.StateMachine.add(
            "FOLLOW_PERSON",
            FollowPerson(),
            transitions={
                "succeeded": "succeeded",
                "lost_person": "failed",
                "timed_out": "failed",
                "failed": "failed",
            },
        )

    # Execute the state machine
    outcome = sm.execute()
    rospy.loginfo(f"State machine completed with outcome: {outcome}")
