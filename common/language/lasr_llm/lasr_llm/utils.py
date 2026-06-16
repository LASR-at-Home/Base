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
    """
    Parse the llm response to get the requested fields into a dict.

    The dictionary keys are the field names, and the values are the result
    from the response.
    :param output: llm response
    :param fields: the fields requested to extract
    :return: dictionary of field names and their values
    """
    field_dict = {field: None for field in fields}
    separator = r"\s*(?:[:=]|\s+-\s+)\s*"
    matches = []

    for field in fields:
        labels = "|".join(_label_pattern(alias) for alias in _field_aliases(field))
        pattern = re.compile(
            rf"(?<![A-Za-z0-9])[\"']?(?:{labels})[\"']?"
            rf"(?![A-Za-z0-9]){separator}",
            re.IGNORECASE,
        )
        matches.extend(
            (match.start(), match.end(), field)
            for match in pattern.finditer(output)
        )

    matches.sort()
    for index, (_, value_start, field) in enumerate(matches):
        line_end = output.find("\n", value_start)
        if line_end == -1:
            line_end = len(output)

        value_end = line_end
        if index + 1 < len(matches):
            value_end = min(value_end, matches[index + 1][0])

        value = output[value_start:value_end].strip(" \t\r\n,;:-\"'`")
        field_dict[field] = value

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
        print(query)
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
