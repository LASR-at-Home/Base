#!/usr/bin/env python3
import os, rospkg, shutil, rospy, sys

rospy.loginfo("setting debug")
DEBUG = rospy.get_param('debug') # 3
rospy.loginfo("setting Debug with images ")
DEBUG_WITH_IMAGES = rospy.get_param('debug_with_images')


# for matplotlib
rospy.loginfo("setting Plot SHOW ")
PLOT_SHOW = rospy.get_param('plot_show')
rospy.loginfo("setting Plot Save ")
PLOT_SAVE = rospy.get_param('plot_save')
rospy.loginfo("setting Publish Markers ")
PUBLISH_MARKERS = rospy.get_param('publish_markers')
rospy.loginfo("setting Debug Path ")
DEBUG_PATH = os.getcwd()
rospy.loginfo("setting Rasa ")
RASA = rospy.get_param('rasa')

rospy.logwarn("DEBUG: {DEBUG}, DEBUG_WITH_IMAGES: {DEBUG_WITH_IMAGES}, PLOT_SHOW: {PLOT_SHOW}, PLOT_SAVE: {PLOT_SAVE}".format(**locals()))

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