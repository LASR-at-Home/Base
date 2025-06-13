import smach
import rospy
import actionlib
from lasr_llm_msgs.srv import (
    StoringGroceriesQueryLlm,
    StoringGroceriesQueryLlmRequest,
)

class ClassifyCategory(smach.State):
    def __init__(self, task):
        super().__init__(
            outcomes=["succeeded", "failed", "empty"],
            input_keys=["table_object", "cabinets_objects", "table_object_category", "cabinet_categories"],
            output_keys=["table_object_category", "cabinet_categories", "cabinet_num"],
        )

        self.task = task #"object" or "cabinet"

        # Define known category sets for safety
        self.category_map = {
            "fruit": {"apple", "banana", "cherry", "dragon fruit", "strawberry", "mango", "orange"},
            "beverage": {"cola", "water", "orange juice", "coffee", "cider", "milk", "juice", },
            "new": {"empty"},
            "clothing": {"hat", "pants", "socks", "t-shirt", "glasses", "trousers", "shoes"},


        }

        rospy.wait_for_service("/storing_groceries/query_llm")
        self.llm_service = rospy.ServiceProxy("/storing_groceries/query_llm", StoringGroceriesQueryLlm)

    def execute(self, userdata):
        if self.task == "object":
            if not userdata.table_object:
                rospy.loginfo("No objects detected.")
                return "empty"

            name = userdata.table_object.get("name", "").lower()

            for category, items in self.category_map.items():
                if name in items:
                    userdata.table_object_category = category
                    rospy.loginfo(f"Found in predefined category: {category}")
                    return "succeeded"
            
            try:
                request = StoringGroceriesQueryLlmRequest()
                request.task = "ClassifyObject"
                request.llm_input = [name]
                response = self.llm_service(request)

                if response.category:
                    userdata.table_object_category = response.category
                    rospy.loginfo(f"LLM returned category: {response.category}")
                    return "succeeded"
                else:
                    rospy.logwarn("LLM returned empty category.")
                    return "failed"
                
            except rospy.ServiceException as e:
                rospy.logerr(f"service call failed: {e}")
                return "failed"
        
        elif self.task == "cabinet":
            if not userdata.cabinets_objects:
                rospy.loginfo("No objects detected.")
                return "empty"
            
            userdata.cabinet_categories = []

            for cabinet_objects in userdata.cabinets_objects:
                names = [obj["name"] for obj in cabinet_objects if "name" in obj]
                found_category = None
                
                for name in names:
                    name = name.lower()
                    for category, items in self.category_map.items():
                        if name in items:
                            found_category = category
                            break
                    if found_category:
                        break
                    
                if found_category:
                    cabinet_category = found_category
                    rospy.loginfo(f"Predefined category found: {cabinet_category}")
                else:
                    try:
                        request = StoringGroceriesQueryLlmRequest()
                        request.task = "ClassifyCabinet"
                        request.llm_input = names
                        response = self.llm_service(request)

                        cabinet_category = response.category if response.category else "unknown"
                        rospy.loginfo(f"LLM returned category: {cabinet_category}")      

                    except rospy.ServiceException as e:
                        rospy.logerr(f"service call failed: {e}")
                        return "failed"
                
                userdata.cabinet_categories.append(cabinet_category)

            return "succeeded"
        
        elif self.task == "link":
            if not userdata.cabinet_categories and not userdata.table_object_category:
                rospy.loginfo("No objects detected.")
                return "empty"
            
            if userdata.table_object_category in userdata.cabinet_categories:
                userdata.cabinet_num = userdata.cabinet_categories.index(userdata.table_object_category)
                return "succeeded"
            else:
                try:
                    request = StoringGroceriesQueryLlmRequest()
                    request.task = "LinkCategory"
                    request.llm_input = [userdata.table_object_category] + userdata.cabinet_categories
                    response = self.llm_service(request)

                    link_category = response.category if response.category and response.category != "nothing" else "new"
                    rospy.loginfo(f"LLM returned category: {link_category}")
                    
                    if link_category in userdata.cabinet_categories:
                        userdata.cabinet_num = userdata.cabinet_categories.index(link_category)
                    else:
                        userdata.cabinet_num = -1 
                        
                    return "succeeded"

                except rospy.ServiceException as e:
                    rospy.logerr(f"service call failed: {e}")
                    return "failed"
    
        else:
            rospy.logerr(f"Unknown task type: {self.task}")
            return "failed"