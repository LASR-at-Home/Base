#!/usr/bin/env python3
import rospy
from lasr_vector_databases_faiss.srv import TxtQuery, TxtQueryRequest

request = TxtQueryRequest()

request.txt_path = (
    "/home/mattbarker/LASR/lasr_ws/src/lasr-base/tasks/gpsr/data/questions.txt"
)

request.index_path = (
    "/home/mattbarker/LASR/lasr_ws/src/lasr-base/tasks/gpsr/data/questions.index"
)

request.query_sentence = "Do French like snails?"

request.k = 3

response = rospy.ServiceProxy("lasr_faiss/txt_query", TxtQuery)(request)

print(response.closest_sentences)
print(response.cosine_similarities)
