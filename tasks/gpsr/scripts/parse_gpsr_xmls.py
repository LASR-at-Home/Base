import xml.etree.ElementTree as ET


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
