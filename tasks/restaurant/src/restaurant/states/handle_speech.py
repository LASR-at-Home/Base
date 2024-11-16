import string
import jellyfish as jf

from typing import List, Dict, Any

SPELLING_THRESHOLD = 1
PRONOUNCIATION_THRESHOLD = 0

excluded_words = [
    "i",
    "would",
    "like",
    "and",
    "please",
    "um"
    "can",
    "have"
    "may",
    "could",
    "get"
]


drinks_list = [
    "cola",
    "ice tea",
    "water",
    "milk",
    "big coke",
    "fanta",
    "dubbelfris"
]

drinks_split_list = [
    "cola",
    "ice",
    "tea",
    "water",
    "milk",
    "big",
    "coke",
    "fanta",
    "dubbelfris"
]

food_list = [
    "cornflakes",
    "pea soup",
    "curry",
    "pancake mix",
    "hagelslag",
    "sausages",
    "mayonaise"
]

food_split_list = [
    "cornflakes",
    "pea", 
    "soup",
    "curry",
    "pancake",
    "mix",
    "hagelslag",
    "sausages",
    "mayonaise"
]

fruits_list = [
    "pear",
    "plum",
    "peach",
    "lemon",
    "orange",
    "strawberry",
    "banana",
    "apple"
]
snacks_list = [
    "stroopwafel",
    "candy",
    "liquorice",
    "crisps",
    "pringles",
    "tictac"
]

items_list = drinks_list + food_list + fruits_list + snacks_list

items_split_list = drinks_split_list + food_split_list + fruits_list + snacks_list

double_word_items_split_list = [    
    "ice",
    "tea",
    "big", 
    "coke",
    "pea",
    "soup",
    "pancake", 
    "mix"
]

double_word_items_full_list = [
    "ice tea",
    "big coke",
    "pea soup",
    "pancake mix"
]

double_word_items_direct_dict = {
    "ice": "tea",
    "tea": "ice",
    "big": "coke",
    "coke": "big",
    "pea": "soup",
    "soup": "pea",
    "pancake": "mix",
    "mix": "pancake"
}

double_word_items_full_dict = {
    "ice": "ice tea",
    "tea": "ice tea",
    "big": "big coke",
    "coke": "big coke",
    "pea": "pea soup",
    "soup": "pea soup",
    "pancake": "pancake mix",
    "mix": "pancake mix"
}


# Enter consecutive numbers in order if more are required
numbers_list = ["a", "one", "two", "three", "four", "five"]
numbers_map = {word: 
    (1 if word in ["a", "one"] 
    else i) 
    for i, word in enumerate(numbers_list, start=0)}

everything_split_list = items_split_list + numbers_list


def main(guest_transcription, last_resort):
    sentence_list = get_split_sentence_list(guest_transcription)
    print(sentence_list)
    num_and_items = get_num_and_items(sentence_list)
    print(num_and_items)

    if num_and_items[0][0] == -1:
        recovered_sentence_list = recover_sentence(sentence_list, num_and_items[0][1], last_resort)
        recovered_num_and_items = get_num_and_items(recovered_sentence_list)
        print(f"Recovered: {recovered_num_and_items}")
        if recovered_num_and_items[0][0] == -1:
            recovered_sentence_list = recover_sentence(sentence_list, num_and_items[0][1], True)
            recovered_num_and_items = get_num_and_items(recovered_sentence_list)
            print(f"Recovered: {recovered_num_and_items}")
            if recovered_num_and_items[0][0] == -1:
                return "unknown"    
        return recovered_num_and_items
    else:
        return num_and_items


def get_split_sentence_list(guest_transcription):
    """Split sentence into a list of strings.

    Args:
        guest_transcription (str): Transcription as a pure string.
    Returns:
        List[str]: Transcription split up as a list of strings.
    """
    filtered_sentence = guest_transcription.lower().translate(
        str.maketrans("", "", string.punctuation)
    )
    sentence_list = [word for word in filtered_sentence.split() if word not in excluded_words]
    return sentence_list


def get_num_and_items(sentence_list):
    """Extract the ordered items and their corresponding number required.

    Args:
        sentence_list (List[str]): Transcription split up as a list of strings.

    Returns:
        List[int][str]: List identifying the pairing of the number words and item words. 
        Error message may be contained.
    """
    number_items = []
    int_number_items = []
    for i in range(len(sentence_list)):
        if sentence_list[i] in numbers_list:
            number_items.append(sentence_list[i])
            int_number_items.append(numbers_map[sentence_list[i]])

    matched_items = []
    for i in range(len(sentence_list)):
        for item in items_list:
            item_words = item.split()
            if sentence_list[i:i + len(item_words)] == item_words:
                matched_items.append(item)
                break

    if len(int_number_items) > len(matched_items):
        return [(-1, "more number")]
    elif len(int_number_items) < len(matched_items):
        return [(-1, "more items")]
    elif len(int_number_items) == 0 and len(matched_items) == 0:
        return [(-1, "unknown")]
    else:
        num_and_items = list(zip(int_number_items, matched_items))
        return (num_and_items)


def recover_sentence(sentence_list, error_message, last_resort):
    """Recover words and phrases in the sentence. If there's a mismatch between the number of number words and item
    words, recover until they are of the same number. If last resort mode is True, then disregard the mismatch in 
    numbers and perform all recovery behaviours.

    Args:
        sentence_list (List[str]): Transcription split up as a list of strings.
        error_message (str): Identifies whether more number or item words were identified.
        last_resort (bool): Whether all recovery behaviour needs to be performed.

    Returns:
        List[str]: Recovered sentence list.
    """
    # Recover items
    if not last_resort and error_message == "more number":
        recovered_sentence = handle_similar_spelt(sentence_list, items_split_list, SPELLING_THRESHOLD)
        # print(f"Recovered items (spelt): {recovered_sentence}")
        recovered_sentence = handle_similar_sound(recovered_sentence, items_split_list, PRONOUNCIATION_THRESHOLD)
        # print(f"Recovered items (sound): {recovered_sentence}")
        recovered_sentence = infer_second_item(recovered_sentence)
        # print(f"Recovered items (inferred second word): {recovered_sentence}")
        return recovered_sentence
    # Recover numbers
    elif not last_resort and error_message == "more items":
        recovered_sentence = handle_similar_spelt(sentence_list, numbers_list, SPELLING_THRESHOLD)
        # print(f"Recovered number (spelt): {recovered_sentence}")
        recovered_sentence = handle_similar_sound(recovered_sentence, numbers_list, PRONOUNCIATION_THRESHOLD)
        # print(f"Recovered number (sound): {recovered_sentence}")
        return recovered_sentence
    else:
        recovered_sentence = handle_similar_spelt(sentence_list, numbers_list, SPELLING_THRESHOLD)
        # print(f"Recovered number (spelt): {recovered_sentence}")
        recovered_sentence = handle_similar_sound(recovered_sentence, numbers_list, PRONOUNCIATION_THRESHOLD)
        # print(f"Recovered number (sound): {recovered_sentence}")

        recovered_sentence = handle_similar_spelt(sentence_list, items_split_list, SPELLING_THRESHOLD)
        # print(f"Recovered items (spelt): {recovered_sentence}")
        recovered_sentence = handle_similar_sound(recovered_sentence, items_split_list, PRONOUNCIATION_THRESHOLD)
        # print(f"Recovered items (sound): {recovered_sentence}")
        recovered_sentence = infer_second_item(recovered_sentence)
        # print(f"Recovered items (inferred second word): {recovered_sentence}")

        return recovered_sentence

def handle_similar_spelt(
    sentence_list: List[str],
    available_words: List[str],
    distance_threshold: int,
) -> str:
    """Recover any word by spelling that has a similarity lower than the specified threshold
    when comparing each word in the sentence list to the list of available words. Replace the old word
    in the sentence with the newly recovered word.

    Args:
        sentence_list (List[str]): Transcription split up as a list of strings.
        available_words (List[str]): List of available words to compare to (numbers or items).
        distance_threshold (int): Similarity in terms of spelling distance required for a word to be recovered.

    Returns:
        List[str]: Recovered sentence in terms of spelling.
    """
    input_word_index = 0

    for input_word in sentence_list:
        input_word = sentence_list[input_word_index]
        for available_word in available_words:
            distance = get_damerau_levenshtein_distance(
                input_word, available_word
            )
            if input_word != available_word and input_word not in everything_split_list:
                if distance <= distance_threshold:
                    print(f"Spelling recovery: {input_word} -> {available_word}")
                    sentence_list[input_word_index] = available_word
        input_word_index = input_word_index + 1
    return sentence_list

def handle_similar_sound(
    sentence_list: List[str],
    available_words: List[str],
    distance_threshold: int,
) -> str:
    """Recover any word by pronounciation that has a similarity lower than the specified threshold
    when comparing each word in the sentence list to the list of available words.

    Args:
        sentence_list (List[str]): Transcription split up as a list of strings.
        available_words (List[str]): List of available words to compare to (numbers or items).
        distance_threshold (int): Similarity in terms of pronounciation distance required for a word to be recovered.

    Returns:
        List[str]: Recovered sentence in terms of pronounciation.
    """
    input_word_index = 0

    for input_word in sentence_list:
        input_word = sentence_list[input_word_index]
        for available_word in available_words:
            distance = get_levenshtein_soundex_distance(
                input_word, available_word
            )
            if input_word != available_word and input_word not in everything_split_list:
                if distance <= distance_threshold:
                    print(f"Sound recovery: {input_word} -> {available_word}")
                    sentence_list[input_word_index] = available_word
        input_word_index = input_word_index + 1
    return sentence_list

def infer_second_item(sentence_list: List[str]) -> str:
    """Infer the second word of any two-worded item phrase in the sentence - if
    a word is contained in any of the two-worded item phrases it is recovered. If the word in the next/previous index
    is the expected second word of a two-worded item, then ignore. If a given phrase has already been recovered, ignore.

    Args:
        sentence_list (List[str]): Transcription split up as a list of strings.

    Returns:
        List[str]: Recovered sentence with the second item inferred.
    """
    allowed_recovery_phrases = list(double_word_items_full_list)
    input_word_index = 0

    for input_word in sentence_list:
        for available_word in double_word_items_split_list:
            if input_word == available_word and double_word_items_full_dict[input_word] in allowed_recovery_phrases:
                recovered_phrase = double_word_items_full_dict[input_word]
                if input_word_index == 0 and sentence_list[input_word_index + 1] != double_word_items_direct_dict[input_word]:
                    print(f"Infer double word: {input_word} -> {recovered_phrase}")
                    allowed_recovery_phrases.remove(recovered_phrase)
                    recovered_phrase_list = get_split_sentence_list(recovered_phrase)
                    sentence_list[input_word_index:input_word_index + 1] = recovered_phrase_list
                elif input_word_index == len(sentence_list) -1 and sentence_list[input_word_index - 1] != double_word_items_direct_dict[input_word]:
                    print(f"Infer double word: {input_word} -> {recovered_phrase}")
                    allowed_recovery_phrases.remove(recovered_phrase)
                    recovered_phrase_list = get_split_sentence_list(recovered_phrase)
                    sentence_list[input_word_index:input_word_index + 1] = recovered_phrase_list  
                elif sentence_list[input_word_index - 1] != double_word_items_direct_dict[input_word] and sentence_list[input_word_index + 1] != double_word_items_direct_dict[input_word]:
                    print(f"Infer double word: {input_word} -> {recovered_phrase}")
                    allowed_recovery_phrases.remove(recovered_phrase)
                    recovered_phrase_list = get_split_sentence_list(recovered_phrase)
                    sentence_list[input_word_index:input_word_index + 1] = recovered_phrase_list
        input_word_index = input_word_index + 1
    return sentence_list


def get_damerau_levenshtein_distance(word_1: str, word_2: str) -> int:
    """Get the damerau-levenshtein distance between two words for the similarity in spelling.

    Args:
        word_1 (str): First word
        word_2 (str): Second word

    Returns:
        int: Damerau-levenshtein distance between the two words.
    """
    return jf.damerau_levenshtein_distance(word_1, word_2)

def get_levenshtein_soundex_distance(word_1: str, word_2: str) -> int:
    """Get the levenshtein distance between the soundex encoding of two words for the similarity
    in pronounciation.

    Args:
        word_1 (str): First word
        word_2 (str): Second word

    Returns:
        int: Levenshtein distance between the soundex encoding of the two words.
    """
    soundex_word_1 = jf.soundex(word_1)
    soundex_word_2 = jf.soundex(word_2)
    return jf.levenshtein_distance(soundex_word_1, soundex_word_2)



# Recover only:
# - User said it's the wrong order (this handles when the number of numbers and 
# items happen to be the same even though something hasn't been picked up) - enters last resort
# - Mismatch between the number of number keywords and the number of item keywords 
# (investigate whether there's more one kind of keyword than the other)

if __name__ == "__main__":
    # guest_transcription = "Hello, are you okay. I would like a curry, three bottles of big coke and one packet of stroopwafel please"
    guest_transcription = "Hello, are you okay. I would like a curry, too bottles of big coke and one packet of stroopwafel please"
    print(main(guest_transcription, False))
    print("====================================================================")
    guest_transcription = "Hello, are you okay. I would like a curry, two bottles of big and one packet of stroopwafel please"
    print(main(guest_transcription, False))
    print("====================================================================")
    guest_transcription = "I would like a curry, two bottles of big uh uh coke and one packet of stroopwafel please"
    print(main(guest_transcription, False))
    print("====================================================================")
    guest_transcription = "I would like a curry, two bottles of big uh uh coke and one packet of stroop waffle please"
    print(main(guest_transcription, False))
    print("====================================================================")
    guest_transcription = "Hello, are you okay. I would like for curry, two ice and one pancake please"
    print(main(guest_transcription, True))
    print("====================================================================")
    guest_transcription = "Hello, are you okay. I would like for curry, too ice and on pancake please"
    print(main(guest_transcription, False))
    print("====================================================================")
    guest_transcription = "Hello, are you okay. I would like for curry, two ice and one pancake please"
    print(main(guest_transcription, False))




