import re
from typing import Optional, List, Dict


def _normalise_field_name(field: str) -> str:
    field = field.lower().replace("_", " ").replace("favorite", "favourite")
    return " ".join(field.split())


def _field_aliases(field: str) -> List[str]:
    aliases = [field]
    normalised = _normalise_field_name(field)

    if normalised == "favourite drink":
        aliases += ["Favorite drink", "Favourite_drink", "Favorite_drink", "Drink"]
    elif normalised == "interests":
        aliases.append("Interest")

    return sorted(set(aliases), key=len, reverse=True)


def _label_pattern(label: str) -> str:
    words = re.split(r"[\s_]+", label.strip())
    return r"[\s_]+".join(re.escape(word) for word in words if word)

def parse_llm_output_to_dict(output: str, fields: List[str]) -> Dict:
    field_dict = {field: None for field in fields}
    separator = re.compile(r"\s*(?:[:=]|\s+-\s+)\s*")
    # print(f"DEBUG: Parsing LLM output:\n{output}")

    for line in output.splitlines():
        sep_match = separator.search(line)
        if not sep_match:
            continue


        key = line[:sep_match.start()].strip().strip("\"'`")
        value = line[sep_match.end():].strip().strip(" \t\r\n,;:-\"'`")

        for field in fields:
            if any(key.lower() == alias.lower() for alias in _field_aliases(field)):
                field_dict[field] = value
                break
    return field_dict

def truncate_llm_output(output: str) -> str:
    """
    If the output is too long, truncate it after the first sentence.

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
        field_str = "\n".join([f"- {field}" for field in fields]) # "\n- Name"
        query = (
            "Extract the following fields from the sentence:\n"
            f"{field_str}\n\n"
            "Return only one line per requested field using this format:\n"
            "Field: value\n"
            "Do not list the requested fields. If a value is missing, leave "
            "it empty after the colon.\n\n"
            "For example, the sentence 'my favourite drink is coca cola' "
            "should return:\n"
            "Favourite drink: coca cola\n\n"
            f"Sentence: {text}."
        )
        # print(f"DEBUG query sent to LLM: {query}")
    elif task == "interest_commonality":
        query = (
            "Extract the commonality (if it exists) of the following "
            f"interests of two people:\n\nSentences: {text}.\n\n"
            "For example, the sentences 'I like football' and 'I like "
            "basketball' should have the commonality 'you both like sports'. "
            "If there is no common interest, say 'you have no common "
            "interests'\n\n"
        )
        # query = (
        #     "Extract the commonality (if it exists) of the following "
        #     f"interests:\n\nInterests: {text}.\nFormat it as a sentence: "
        #     "'you both have interests which are...'"
        # )
    else:
        raise ValueError(
            f"Unknown task: {task}. Supported tasks are 'extract_fields' and "
            "'interest_commonality'."
        )

    return query
