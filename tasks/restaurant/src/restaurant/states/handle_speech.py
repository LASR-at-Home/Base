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

items_list = list(set(drinks_list) | set(food_list) | set(fruits_list) | set(snacks_list))

def main(guest_transcription):

    filtered_sentence = guest_transcription.lower().translate(
        str.maketrans("", "", string.punctuation)
    )
    sentence_list = [word for word in filtered_sentence.split() if word not in excluded_words]

    print(sentence_list)



if __name__ == "__main__":
    guest_transcription = "Hello, are you okay"
    main(guest_transcription)
