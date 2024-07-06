import os
from typing import List
import rospy
import rospkg
from lasr_vector_databases_msgs.srv import TxtIndex, TxtIndexRequest


def create_txt_file(output_path: str) -> None:
    """Creates a txt file containing all permutations of
    "My name is <name> and my favorite drink is <drink>"

    Args:
        output_path (str): Path to the output txt file
    """

    names: List[str] = rospy.get_param("/priors/names")
    drinks: List[str] = rospy.get_param("/priors/drinks")

    with open(output_path, "w") as f:
        for name in names:
            for drink in drinks:
                f.write(f"My name is {name} and my favorite drink is {drink}\n")


def create_vector_db(txt_path: str, output_path: str) -> None:
    """Creates a vector database from a txt file containing
    all permutations of "My name is <name> and my favorite drink is <drink>"

    Args:
        txt_path (str): Path to the txt file
        output_path (str): Path to the output vector database
    """

    rospy.wait_for_service("lasr_faiss/txt_index")
    txt_index = rospy.ServiceProxy("lasr_faiss/txt_index", TxtIndex)

    request = TxtIndexRequest()
    request.txt_paths = [txt_path]
    request.index_paths = [output_path]
    request.index_factory_string = "Flat"
    txt_index(request)


if __name__ == "__main__":
    data_dir = os.path.join(rospkg.RosPack().get_path("receptionist"), "data")
    txt_path = os.path.join(data_dir, "name_and_drink_vector_db.txt")
    output_path = os.path.join(data_dir, "name_and_drink.index")
    create_vector_db(txt_path, output_path)
