#!/usr/bin/env python3
import os, rospkg, shutil

DEBUG = 3
DEBUG_WITH_IMAGES = True


# for matplotlib
PLOT_SHOW = False
PLOT_SAVE = True
DEBUG_PATH = os.getcwd()

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