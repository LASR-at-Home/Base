#!/usr/bin/env python3

import rospy
import smach
import xml.etree.ElementTree as ET

from lasr_vector_databases_msgs.srv import TxtQuery, TxtQueryRequest


def parse_question_xml(xml_file_path: str) -> dict:
    """Parses the GPSR Q/A xml file and returns a dictionary
    consisting of two lists, one for questions and one for answers,
    where the index of each question corresponds to the index of its
    corresponding answer.

    Args:
        xml_file_path (str): full path to xml file to parse

    Returns:
        dict: dictionary with keys "questions" and "answers"
        each of which is a list of strings.
    """
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    parsed_questions = []
    parsed_answers = []
    for q_a in root:
        question = q_a.find("q").text
        answer = q_a.find("a").text
        parsed_questions.append(question)
        parsed_answers.append(answer)

    return {"questions": parsed_questions, "answers": parsed_answers}


class XmlQuestionAnswer(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["query_sentence", "k", "index_path", "txt_path", "xml_path"],
            output_keys=["closest_answers"],
        )
        self.txt_query = rospy.ServiceProxy("/lasr_faiss/txt_query", TxtQuery)

    def execute(self, userdata):
        rospy.wait_for_service("/lasr_faiss/txt_query")
        q_a_dict: dict = parse_question_xml(userdata.xml_path)
        try:
            request = TxtQueryRequest(
                userdata.txt_path,
                userdata.index_path,
                userdata.query_sentence,
                userdata.k,
            )
            result = self.txt_query(request)
            answers = [
                q_a_dict["answers"][q_a_dict["questions"].index(q)]
                for q in result.closest_sentences
            ]
            userdata.closest_answers = answers
            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform Index Query. ({str(e)})")
            userdata.closest_answers = []
            return "failed"
