import string

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

food_list = [
    "cornflakes",
    "pea soup",
    "curry",
    "pancake mix",
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

# Enter consecutive numbers in order if more are required
numbers_list = ["a", "one", "two", "three", "four", "five"]
numbers_map = {word: 
    (1 if word in ["a", "one"] 
    else i) 
    for i, word in enumerate(numbers_list, start=0)}

def get_split_sentence_list(guest_transcription):
    filtered_sentence = guest_transcription.lower().translate(
        str.maketrans("", "", string.punctuation)
    )
    sentence_list = [word for word in filtered_sentence.split() if word not in excluded_words]
    return sentence_list


def get_num_and_items(sentence_list):

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

def recover_sentence(sentence_list, error_message):
    if error_message == "more numnber":
        pass
    elif error_message == "more items":
        pass
    else:
        pass



# Recover only:
# - User said it's the wrong order
# - Mismatch between the number of number keywords and the number of item keywords (investigate whether there's more one kind of keyword than the other)

if __name__ == "__main__":
    # guest_transcription = "Hello, are you okay. I would like a curry, three bottles of big coke and one packet of stroopwafel please"
    guest_transcription = "Hello, are you okay. I would like a curry, too bottles of big coke and one packet of stroopwafel please"
    sentence_list = get_split_sentence_list(guest_transcription)
    print(sentence_list)
    num_and_items = get_num_and_items(sentence_list)
    print(num_and_items)

    if num_and_items[0][0] == -1:
        recovered_sentence_list = recover_sentence(sentence_list, num_and_items[0][1])



