#!/usr/bin/env python3
import rospy


if __name__ == "__main__":
    rospy.init_node("test_index_service")
    rospy.wait_for_service("lasr_faiss/txt_index")
    from lasr_vector_databases_msgs.srv import TxtIndex, TxtIndexRequest

    request = TxtIndexRequest()
    txt_paths = []
    for i in range(20):
        txt_paths.append(
            f"/home/mattbarker/robot_club/lasr_ws/src/Base/tasks/gpsr/data/command_data/all_commands_german_open.txt_chunk_{i+1}.txt"
        )

    request.txt_paths = txt_paths
    request.index_paths = [
        "/home/mattbarker/robot_club/lasr_ws/src/Base/tasks/gpsr/data/faiss_indices/all_gpsr_commands_german_open.index"
    ]
    request.index_factory_string = "IVF4096,PQ32"

    rospy.ServiceProxy("lasr_faiss/txt_index", TxtIndex)(request)
    rospy.spin()
