import rospy
import rosparam
from tiago_controllers import BaseController, HeadController
from lasr_voice import Voice
from play_motion_msgs.msg import PlayMotionAction
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform
from lasr_shapely import LasrShapely
from lasr_speech.srv import Speech
import actionlib
import yaml



class Context:

    def __init__(self, config_path):
        self.base_controller = BaseController()
        self.head_controller = HeadController()
        self.voice_controller = Voice()
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        self.tf = rospy.ServiceProxy("/tf_transform", TfTransform)
        self.shapely = LasrShapely()
        self.speech = rospy.ServiceProxy("/lasr_speech/transcribe_and_parse", Speech)

        self.tables = dict()

        with open(config_path, "r") as fp:
            data = yaml.safe_load(fp)

        self.tables = {
            table: {"status" : "unvisited", "people": list(), "order": list()} for table in data["tables"].keys()
        }

        self.target_objects = data["objects"]

        rosparam.delete_param("tables")
        rosparam.load_file(config_path)

        self.current_table = None