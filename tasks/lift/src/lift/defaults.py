#!/usr/bin/env python3
import os, rospkg, shutil, rospy, sys

DEBUG = rospy.get_param('debug') # 3
DEBUG_WITH_IMAGES = rospy.get_param('debug_with_images')


# for matplotlib
PLOT_SHOW = rospy.get_param('plot_show')
PLOT_SAVE = rospy.get_param('plot_save')
DEBUG_PATH = os.getcwd()

rospy.logwarn(f"DEBUG: {DEBUG}, DEBUG_WITH_IMAGES: {DEBUG_WITH_IMAGES}, PLOT_SHOW: {PLOT_SHOW}, PLOT_SAVE: {PLOT_SAVE}")

if DEBUG_WITH_IMAGES:
    if not os.path.exists(os.path.join(rospkg.RosPack().get_path("lift"), "debug_lift")):
        os.mkdir(os.path.join(rospkg.RosPack().get_path("lift"), "debug_lift"))
    DEBUG_PATH = os.path.join(rospkg.RosPack().get_path("lift"), "debug_lift",)
    if os.path.exists(DEBUG_PATH):
        shutil.rmtree(DEBUG_PATH)
    os.mkdir(DEBUG_PATH)


# for plots in waypoitns
import random
TEST = random.randint(0, 1000)




# if self.debug:
#             cv2.imwrite(os.path.join(self.debug_path, f"objects_{self.num_detections}_mask.png"), objects_mask)
#             cv2.imwrite(os.path.join(self.debug_path, f"objects_{self.num_detections}.png"), raw_im)
#             with open(os.path.join(self.debug_path, f"objects_{self.num_detections}.txt"), "w+") as fp:
#                 for detection in detections_objects_:
#                     fp.write(f"{detection.name}\n")


# from lift.defaults import PLOT_SAVE, PLOT_SHOW, TEST, DEBUG_PATH
# if PLOT_SHOW:
#     plt.show()
# if PLOT_SAVE:
#     plt.savefig(DEBUG_PATH + "/dilation" + str(TEST) + ".jpg")