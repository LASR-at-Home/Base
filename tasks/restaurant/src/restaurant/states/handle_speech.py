import string

excluded_words = [
    "my",
    "name",
    "is",
    "and",
    "favourite",
    "drink",
    "you",
    "can",
    "call",
    "me",
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

numbers_list = ["a", "one", "two", "three", "four", "five"]

def main(guest_transcription):

    filtered_sentence = guest_transcription.lower().translate(
        str.maketrans("", "", string.punctuation)
    )
    sentence_list = [word for word in filtered_sentence.split() if word not in excluded_words]

    print(sentence_list)


    number_items = []
    for i in range(len(sentence_list)):
        if sentence_list[i] in numbers_list:
            number_items.append(sentence_list[i])

    print(number_items)

    matched_items = []
    for i in range(len(sentence_list)):
        for item in items_list:
            item_words = item.split()
            if sentence_list[i:i + len(item_words)] == item_words:
                matched_items.append(item)
                break 
    
    print(matched_items)



if __name__ == "__main__":
    guest_transcription = "Hello, are you okay. I would like a curry, three big coke and one stroopwafel"
    main(guest_transcription)
