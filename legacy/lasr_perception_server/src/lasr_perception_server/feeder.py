import rospy
from sensor_msgs.msg import Image, PointCloud2

# taking multiple image raws and sending them to perception server
#TODO: work with pcl as well
def looking_around_and_taking_imgs(head_controller):
    pcls = []
    direction = 0.2
    images = []

    for _ in range(4):
        direction *= -1
        head_controller.sync_reach_to(direction, -0.2, velocities=[0.1, 0.0])
        # pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        # pcls.append(pcl_msg)
         # Choose topic
        if rospy.get_published_topics(namespace='/xtion'):
            topic = '/xtion/rgb/image_raw'
        else:
            topic = '/usb_cam/image_raw'
        image_msg = rospy.wait_for_message(topic, Image)
        images.append(image_msg)
        print(len(images))
    return images

