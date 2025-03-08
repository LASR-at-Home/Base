import os
import smach
import rclpy
from rclpy.node import Node
from typing import Optional
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from rclpy.task import Future
from lasr_skills import AccessNode


class GetImage(smach.State):
    """
    State for reading an sensor_msgs Image message
    """

    def __init__(self, topic: Optional[str] = None):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["img_msg"], input_keys=["img_msg"] 
        )
        self.node = AccessNode.get_node()

        self.topic = topic or "/xtion/rgb/image_raw"
        # self.topic = topic or "/image_raw"
        # TODO check if tiago is in environment
        #else "/usb_cam/image_raw", self.topic = topic

    def execute(self, userdata):
        if not rclpy.ok():
            rclpy.init()

        try:
            msg = self.node.wait_for_message(self.topic, Image)
            if msg is not None:
                userdata.img_msg = msg
            else:
                userdata.img_msg = None
            if userdata.img_msg is None:
                return "failed"
            
        except Exception as e:
            self.node.get_logger().error(str(e))
            return "failed"
        finally:
            if self.node is not None:
                self.node.destroy_node()
        print(userdata.img_msg)
        return "succeeded"


class GetPointCloud(smach.State):
    """
    State for acquiring a PointCloud2 message.
    """

    def __init__(self, topic: Optional[str] = None):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["pcl_msg"], input_keys=["pcl_msg"]
        )
        self.node = AccessNode.get_node()

        self.topic = topic or "/xtion/depth_registered/pints"

    def execute(self, userdata):
        if not rclpy.ok():
            rclpy.init()
        try:
            userdata.pcl_msg = None
            userdata.pcl_msg = self.node.wait_for_message(self.topic, PointCloud2)
            if userdata.pcl_msg is None:
                return "failed"
        except Exception as e:
            self.node.get_logger().error(str(e))
            return "failed"
        finally:
            if self.node is not None:
                self.node.destroy_node()

        return "succeeded"


class GetImageAndPointCloud(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["img_msg", "pcl_msg"], input_keys=["img_msg", "pcl_msg"]
        )
        self.node = AccessNode.get_node()

        self.topic1 = "/xtion/rgb/image_raw"
        self.topic2 = "/xtion/depth_registered/points"

        self.topic1 = "/xtion/rgb/image_raw"
        self.topic2 = "/xtion/depth_registered/points"

    def execute(self, userdata):
        if not rclpy.ok():
            rclpy.init()
        try:
            userdata.img_msg = self.node.wait_for_message(self.topic1, Image)
            userdata.pcl_msg = self.node.wait_for_message(self.topic2, PointCloud2)

            if userdata.img_msg is None or userdata.pcl_msg is None:
                return "failed"
        except Exception as e:
            self.node.get_logger().error(str(e))
            return "failed"
        finally:
            if self.node is not None:
                self.node.destroy_node()

        return "succeeded"
    

# class ROS2HelperNode(Node):
#     def __init__(self, name="ros2_helper_node"):
#         super().__init__(name)

#     def wait_for_message(self, topic, msg_type, timeout=5.0):
#         """
#         ROS2 does not provide wait_for_message
#         Waits for a message with a Future object (More efficient). 
#         """
#         future = Future()
#         qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

#         def callback(msg):
#             if not future.done():
#                 future.set_result(msg)

#         self.subscriber = self.create_subscription(msg_type, topic, callback, qos_profile)
#         # self.subscriber.append(sub)


#         start_time = self.get_clock().now().nanoseconds / 1e9

#         while not future.done():
#             rclpy.spin_once(self, timeout_sec=0.1)
#             elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - start_time
#             if elapsed_time > timeout:
#                 return None

#         return future.result()

# class GetImage(smach.State):
#     """
#     State for reading an sensor_msgs Image message
#     """

#     def __init__(self, topic: Optional[str] = None):
#         smach.State.__init__(
#             self, outcomes=["succeeded", "failed"], output_keys=["img_msg"], input_keys=["img_msg"] 
#         )

#         self.topic = topic or "/xtion/rgb/image_raw"
#         # self.topic = topic or "/image_raw"
#         # TODO check if tiago is in environment
#         #else "/usb_cam/image_raw", self.topic = topic

#     def execute(self, userdata):
#         if not rclpy.ok():
#             rclpy.init()

#         node = None

#         try:
#             node = ROS2HelperNode()
#             msg = node.wait_for_message(self.topic, Image)
#             if msg is not None:
#                 userdata.img_msg = msg
#             else:
#                 userdata.img_msg = None
#             if userdata.img_msg is None:
#                 return "failed"
            
#         except Exception as e:
#             node.get_logger().error(str(e))
#             return "failed"
#         finally:
#             if node is not None:
#                 node.destroy_node()
#         print(userdata.img_msg)
#         return "succeeded"

# class GetPointCloud(smach.State):
#     """
#     State for acquiring a PointCloud2 message.
#     """

#     def __init__(self, topic: Optional[str] = None):
#         smach.State.__init__(
#             self, outcomes=["succeeded", "failed"], output_keys=["pcl_msg"], input_keys=["pcl_msg"]
#         )
        
#         self.topic = topic or "/xtion/depth_registered/pints"

#     def execute(self, userdata):
#         if not rclpy.ok():
#             rclpy.init()

#         node = None

#         try:
#             node = ROS2HelperNode()
#             userdata.pcl_msg = None
#             userdata.pcl_msg = node.wait_for_message(self.topic, PointCloud2)
#             if userdata.pcl_msg is None:
#                 return "failed"
#         except Exception as e:
#             node.get_logger().error(str(e))
#             return "failed"
#         finally:
#             if node is not None:
#                 node.destroy_node()

#         return "succeeded"

# class GetImageAndPointCloud(smach.State):
#     def __init__(self):
#         smach.State.__init__(
#             self, outcomes=["succeeded", "failed"], output_keys=["img_msg", "pcl_msg"], input_keys=["img_msg", "pcl_msg"]
#         )

#         self.topic1 = "/xtion/rgb/image_raw"
#         self.topic2 = "/xtion/depth_registered/points"

#         self.topic1 = "/xtion/rgb/image_raw"
#         self.topic2 = "/xtion/depth_registered/points"

#     def execute(self, userdata):
#         if not rclpy.ok():
#             rclpy.init()

#         node = None
        
#         try:
#             node = ROS2HelperNode()

#             userdata.img_msg = node.wait_for_message(self.topic1, Image)
#             userdata.pcl_msg = node.wait_for_message(self.topic2, PointCloud2)

#             if userdata.img_msg is None or userdata.pcl_msg is None:
#                 return "failed"
#         except Exception as e:
#             node.get_logger().error(str(e))
#             return "failed"
#         finally:
#             if node is not None:
#                 node.destroy_node()

#         return "succeeded"

# def main(args=None):
#     rclpy.init(args=args)

#     # rclpy.init('smach_example_state_machine')

#     sm = smach.StateMachine(outcomes=['failed','succeeded'])
#     with sm:
#         # smach.StateMachine.add('GetImage', GetImage(),
#         #     transitions={'failed': 'failed', 'succeeded':'succeeded'},
#         # )
#         # smach.StateMachine.add('GetPointCloud', GetPointCloud(),
#         #     transitions={'failed': 'failed', 'succeeded': 'succeeded'},
#         # )
#         smach.StateMachine.add('GetImageAndPointCloud', GetImageAndPointCloud(),
#             transitions={'failed': 'failed', 'succeeded': 'succeeded'},
#         )
    
#     outcome = sm.execute()


# if __name__ == '__main__':
#     main()