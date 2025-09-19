import smach
import rospy
import json
from collections import Counter
import difflib
from play_motion_msgs.msg import PlayMotionGoal
from control_msgs.msg import PointHeadGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PoseWithCovarianceStamped
# from coffee_shop_ui.msg import Order
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal
from scipy.spatial.transform import Rotation as R

# TODO cannot order from tablet now, add if needed

class TakeOrder(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done"])
        self.context = context
        self.tablet_pub = rospy.Publisher("/tablet/screen", String, queue_size=10)

    def listen(self):
        resp = self.context.listen()
        if resp is None:
            self.context.say(
                self.context.get_random_retry_utterance()
            )
            return self.listen()
        # resp = json.loads(resp)
        rospy.loginfo(resp)
        return resp

    def get_order(self):
        resp = self.listen()
        resp = self.parse_response(resp)
        if resp["intent"]["name"] != "make_order":
            rospy.logwarn("The intent was wrong")
            self.context.say(
                self.context.get_random_retry_utterance()
            )
            return self.get_order()
        items = resp["entities"].get("item", [])
        if not items:
            rospy.logwarn("There were no items")
            self.context.say(
                self.context.get_random_retry_utterance()
            )
            return self.get_order()
        quantities = resp["entities"].get("CARDINAL", [])
        quantified_items = []
        if len(items) == len(quantities) == 1:
            quantified_items.append((int(quantities[0]["value"]), items[0]["value"]))
        else:
            for item in items:
                quantified = False
                for quantity in quantities:
                    if quantity["end"] == item["start"] - 1:
                        quantified_items.append((int(quantity["value"]), item["value"]))
                        quantified = True
                        break
                if not quantified:
                    quantified_items.append((1, item["value"]))
        items = []
        for quantity, item in quantified_items:
            if item not in self.context.target_object_remappings.keys():
                matches = difflib.get_close_matches(
                    item.lower(), self.context.target_object_remappings.keys()
                )
                if matches:
                    item = matches[0]
                else:
                    continue
            items.extend([item.lower()] * quantity)
        if not items:
            self.context.say(
                self.context.get_random_retry_utterance()
            )
            return self.get_order()
        return items

    def parse_response(self, resp: str):
        response = {"intent": {"name": ""}, "entities": {}}
        if resp:
            response["intent"]["name"] = "make_order"
            entities = []
            for word in resp.split():
                if word.lower() in self.context.target_object_remappings.keys():
                    entities.append({"entity": "item", "value": word.lower()})
                elif word.isdigit():
                    entities.append({"entity": "CARDINAL", "value": word})
            response["entities"] = {"item": [], "CARDINAL": []}
            index = 0
            for entity in entities:
                entity["start"] = index
                entity["end"] = index + len(entity["value"]) - 1
                index += len(entity["value"]) + 1
                response["entities"][entity["entity"]].append(entity)
            return response

    def affirm(self):
        resp = self.listen()
        resp = self.parse_affirmation(resp)
        if resp["intent"]["name"] not in ["affirm", "deny"]:
            self.context.say(
                self.context.get_random_retry_utterance()
            )
            return self.affirm()
        return resp["intent"]["name"] == "affirm"

    def parse_affirmation(self, resp: str):
        response = {"intent": {"name": ""}}
        if resp:
            resp = resp.lower()
            if "yes" in resp or "yeah" in resp or "yep" in resp or "correct" in resp:
                response["intent"]["name"] = "affirm"
            elif "no" in resp or "nope" in resp or "nah" in resp or "incorrect" in resp:
                response["intent"]["name"] = "deny"
        return response

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")

        if self.context.tablet:
            self.context.say(
                "Please use the tablet to make your order."
            )
            if self.context.tablet_on_head:
                pm_goal = PlayMotionGoal(motion_name="tablet", skip_planning=True)
                self.context.play_motion_client.send_goal_and_wait(pm_goal)
                rospy.loginfo("Tablet is on the head")
            else:
                rospy.loginfo("Tablet is not on the head")
                robot_pose = rospy.wait_for_message(
                    "/amcl_pose", PoseWithCovarianceStamped
                ).pose.pose
                target_orientation = R.from_quat(
                    [
                        robot_pose.orientation.x,
                        robot_pose.orientation.y,
                        robot_pose.orientation.z,
                        robot_pose.orientation.w,
                    ]
                ) * R.from_euler("z", 180.0, degrees=True)
                move_base_goal = MoveBaseGoal()
                move_base_goal.target_pose.header.frame_id = "map"
                move_base_goal.target_pose.pose = Pose(
                    position=robot_pose.position,
                    orientation=Quaternion(*target_orientation.as_quat()),
                )
                self.context.move_base_client.send_goal_and_wait(move_base_goal)
                pm_goal = PlayMotionGoal(
                    motion_name="tablet_no_head", skip_planning=True
                )
                self.context.play_motion_client.send_goal_and_wait(pm_goal)

            self.tablet_pub.publish(String("order"))
            self.context.say("order")
            resp = self.context.listen()
            if resp is not None:
                rospy.loginfo(resp)
                self.context.say(f"You have ordered {resp}")
            # TODO fix if using tablet
            # order = rospy.wait_for_message("/tablet/order", Order).products
        else:
            ph_goal = PointHeadGoal()
            ph_goal.max_velocity = 1.0
            ph_goal.pointing_frame = "head_2_link"
            ph_goal.pointing_axis = Point(1.0, 0.0, 0.0)
            ph_goal.target.header.frame_id = "map"
            ph_goal.target.point = Point(*self.context.get_interaction_person())
            pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
            self.context.play_motion_client.send_goal_and_wait(pm_goal)

            if len(self.context.tables[self.context.current_table]["people"]) == 1:
                self.context.point_head_client.send_goal_and_wait(ph_goal)
                self.context.say(
                    "Hello, I'm TIAGo, I'll be serving you today."
                )
                self.context.say(
                    "Please state your order after the beep - this indicates that I am listening."
                )
            elif len(self.context.tables[self.context.current_table]["people"]) == 2:
                self.context.say(
                    "Greetings to both of you, I'm TIAGo, I'll be serving you today."
                )
                self.context.point_head_client.send_goal_and_wait(ph_goal)
                self.context.say(
                    "I choose you to be the one in charge."
                )
                self.context.say(
                    "Please state the order for the two of you after the beep - this indicates that I am listening."
                )
            else:
                self.context.say(
                    "Salutations to all of you, I'm TIAGo, I'll be serving you today."
                )
                self.context.point_head_client.send_goal_and_wait(ph_goal)
                self.context.say(
                    "I choose you to be the one in charge."
                )
                self.context.say(
                    "Please state the order for the group after the beep - this indicates that I am listening."
                )

            order = []

            while True:
                order.extend(self.get_order())

                items_string = ", ".join(
                    [
                        f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}"
                        for item, count in Counter(order).items()
                    ]
                ).replace(", ", ", and ", len(order) - 2)

                self.context.say(
                    f"You asked for {items_string} so far, can I get you anything else? Please answer yes or no after the beep."
                )
                if self.affirm():
                    self.context.say(
                        "Okay, please state the additional items after the beep."
                    )
                else:
                    break
        print(self.context.target_object_remappings)
        print(order)
        order_string = ", ".join(
            [
                f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}"
                for item, count in Counter(order).items()
            ]
        ).replace(", ", ", and ", len(order) - 2)

        self.context.say(f"Your order is {order_string}")
        self.context.tables[self.context.current_table]["order"] = order
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        self.context.start_head_manager("head_manager", "")
        return "done"
