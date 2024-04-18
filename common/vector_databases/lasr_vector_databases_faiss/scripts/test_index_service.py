#!/usr/bin/env python3
import rospy
from lasr_vector_databases_faiss.srv import TxtIndex, TxtIndexRequest

request = TxtIndexRequest()

request.txt_paths = [
    "/home/mattbarker/LASR/lasr_ws/src/lasr-base/tasks/gpsr/data/questions.txt"
]

request.index_paths = [
    "/home/mattbarker/LASR/lasr_ws/src/lasr-base/tasks/gpsr/data/questions.index"
]
rospy.ServiceProxy("lasr_faiss/txt_index", TxtIndex)(request)
