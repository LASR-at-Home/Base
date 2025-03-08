import os
import smach
import cv2_img
import cv2_pcl
import rclpy
# from .get_image import GetImage, ROS2HelperNode, GetPointCloud
# from .image_cv2_to_msg import ImageCv2ToMsg

class ImageMsgToCv2(smach.State):
    """
    State for converting a sensor Image message to cv2 format
    """

    def __init__(self):
        smach.State.__init__(
            # self, outcomes=["succeeded", "failed"], input_keys=["img_msg", "img"], output_keys=["img"]
            self, outcomes=["succeeded", "failed"], input_keys=["img_msg"], output_keys=["img"]
        )

    def execute(self, userdata):
        userdata.img = cv2_img.msg_to_cv2_img(userdata.img_msg)
        # print(userdata.img)
        return "succeeded"
    
class PclMsgToCv2(smach.State):
    """
    State for converting a sensor Image message to cv2 format
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg_3d"],
            output_keys=["img", "xyz"],
        )

    def execute(self, userdata):
        userdata.img = cv2_pcl.pcl_to_cv2(userdata.img_msg_3d)
        userdata.xyz = cv2_pcl.pointcloud2_to_xyz_array(userdata.img_msg_3d)
        return "succeeded"
    
# def main(args=None):
#     rclpy.init(args=args)

#     # rclpy.init('smach_example_state_machine')

#     sm = smach.StateMachine(outcomes=['failed','succeeded'])
#     with sm:
#         smach.StateMachine.add('GetPointCloud', GetPointCloud(),
#             transitions={'failed': 'failed', 'succeeded':'PclMsgToCv2'},
#         )
#         smach.StateMachine.add('PclMsgToCv2', PclMsgToCv2(),
#             transitions={'failed': 'failed', 'succeeded':'succeeded'},
#             remapping={'img_msg_3d': 'pcl_msg'}
#         )
#         # smach.StateMachine.add('GetImage', GetImage(),
#         #     transitions={'failed': 'failed', 'succeeded':'ImageMsgToCv2'},
#         # )
#         # smach.StateMachine.add('ImageMsgToCv2', ImageMsgToCv2(),
#         #     transitions={'succeeded': 'ImageCv2ToMsg', 'failed': 'failed'},
#         # )
#         # smach.StateMachine.add('ImageCv2ToMsg', ImageCv2ToMsg(),
#         #     transitions={'succeeded': 'succeeded', 'failed': 'failed'},
#         # )
        
#     outcome = sm.execute()


# if __name__ == '__main__':
#     main()