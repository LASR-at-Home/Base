import re
from typing import Optional, List, Dict


def parse_llm_output_to_dict(output: str, fields: List[str]) -> Dict:
    """
    Parse the llm response to get the requested fields into a dict where the keys are the field names,
    and the values are the result from the response.
    :param output: llm response
    :param fields: the fields requested to extract
    :return: dictionary of field names and their values
    """
    field_dict = {field: None for field in fields}
    for field in fields:
        pattern = re.compile(rf"{field}:\s*(.*?)(?:\n|$)")  # field: value
        match = pattern.search(output)
        if match:
            field_dict[field] = match.group(1).strip()
    return field_dict


def truncate_llm_output(output: str) -> str:
    """
    If the output is too long, truncate it to a reasonable length, after the first sentence.
    :param output: the output from the LLM
    :return: the parsed output
    """
    sentences = re.split(r"(?<=[.!?]) +", output)
    if len(sentences) > 1:
        return sentences[0]
    else:
        return output


def create_query(text: str, task: str, fields: Optional[List[str]] = None):
    """
    Create a query for the LLM to extract specific fields from the input text.
    :param text: The input sentence to process.
    :param task: the task to perform (extract_fields, interest_commonality)
    :param fields: A list of fields to extract.
    """
    if task == "extract_fields" and fields is None:
        fields = ["Name", "Favourite drink", "Interests"]

    if task == "extract_fields":
        assert (
            fields is not None
        ), "Fields must be provided for the 'extract_fields' task."
        field_str = "\n".join([f"- {field}" for field in fields])
        query = f"Extract the following fields from the sentence:\n{field_str}\n\n For example, the sentence ' my favourite drink is cocal cola' should have the field favourite_drink matched to 'coca cola' \n\n Sentence: {text}."
    elif task == "interest_commonality":
        query = f"Extract the commonality (if it exists) of the following interests of two people:\n\nSentences: {text}. \n\n For example, the sentences 'I like football' and 'I like basketball' should have the commonality 'you both like sports'. If there is no common interest, say 'you have no common interests'\n\n"
        # query = f"Extract the commonality (if it exists) of the following interests:\n\nInterests: {text}.\nFormat it as a sentence: 'you both have interests which are...'"
    else:
        raise ValueError(
            f"Unknown task: {task}. Supported tasks are 'extract_fields' and 'interest_commonality'."
        )

    return query
