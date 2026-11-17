"""Object name -> category -> where it goes. Shared by the task (ClassifyCategory) and the
detect_tuner, so both route objects the same way. The rule is DecideDestination's:
tableware and cutlery ("dish") -> dishwasher, the announced trash category -> trash bin,
everything else -> cabinet."""

CATEGORY_MAP = {
    'fruit': {
        'apple', 'banana', 'orange',
    },
    'vegetable': {
        'broccoli', 'cabbage', 'carrot', 'corn', 'cucumber', 'garlic',
        'lettuce', 'onion', 'pepper', 'potato', 'radish', 'tomato',
        'zucchini',
    },
    'beverage': {
        'bottle', 'coffee cup', 'coke', 'coke can', 'energy drink', 'iced tea',
        'iced tea can', 'juice', 'milk', 'red bull', 'soda can', 'sprite',
        'thermos', 'water bottle',
    },
    'snack': {
        'biscuit', 'biscuits', 'candy', 'chips', 'chocolate bar', 'cookie',
        'crackers', 'crisp packet', 'crisps', 'granola bar', 'popcorn', 'pringles',
        'pringles tube', 'snack bag',
    },
    'cleaning': {
        'broom', 'brush', 'bucket', 'cleaner', 'detergent', 'mop',
        'soap', 'sponge', 'spray bottle', 'tissue box', 'toilet paper', 'toothpaste',
    },
    'cereal': {
        'cereal', 'cereal box', 'granola', 'granola box', 'muesli', 'oats',
    },
    'dish': {
        'bowl', 'chopsticks', 'cup', 'fork', 'knife', 'mug',
        'paper cup', 'plate', 'spoon', 'tea spoon', 'teaspoon', 'wine glass',
    },
}

DISHWASHER_CATEGORIES = {"dish"}
DEST_STR = {"dishwasher": "the dishwasher", "trash_bin": "the trash bin", "cabinet": "the cabinet"}

# how to say a category: (one object, several objects)
CAT_WORDS = {
    "beverage": ("a drink", "drinks"),
    "snack": ("a snack", "snacks"),
    "cereal": ("cereal", "cereal"),
    "dish": ("tableware", "tableware"),
    "fruit": ("fruit", "fruit"),
    "vegetable": ("a vegetable", "vegetables"),
    "cleaning": ("a cleaning product", "cleaning products"),
}


def category_of(name):
    """The CATEGORY_MAP category of an object name, or None."""
    n = (name or "").strip().lower()
    for cat, items in CATEGORY_MAP.items():
        if n in items:
            return cat
    return None


def destination_of(category, trash_category="snack"):
    """'dishwasher', 'trash_bin' or 'cabinet' (None if the category is unknown)."""
    if not category:
        return None
    if category in DISHWASHER_CATEGORIES:
        return "dishwasher"
    if trash_category and category == trash_category.lower():
        return "trash_bin"
    return "cabinet"


def _join(xs):
    return xs[0] if len(xs) == 1 else ", ".join(xs[:-1]) + " and " + xs[-1]


def announcement(names, trash_category="snack"):
    """What the robot says about the objects it sees (names in left-to-right order)."""
    names = list(dict.fromkeys(n.strip() for n in names if n and n.strip()))
    if not names:
        return "I can't see any objects on the table."
    art = lambda n: ("an " if n[0].lower() in "aeiou" else "a ") + n
    out = ["I can see " + _join([art(n) for n in names]) + "."]
    groups = {}
    for n in names:                      # grouped by category, in order of first appearance
        groups.setdefault(category_of(n), []).append(n)
    for cat, ns in groups.items():
        the = _join(["the " + n for n in ns])
        if cat is None:
            out.append(f"I don't know where {the} {'go' if len(ns) > 1 else 'goes'}.")
            continue
        one, many = CAT_WORDS.get(cat, (cat, cat))
        dest = DEST_STR[destination_of(cat, trash_category)]
        if len(ns) == 1:
            out.append(f"{the[0].upper() + the[1:]} is {one}, so it goes in {dest}.")
        else:
            out.append(f"{the[0].upper() + the[1:]} are {many}, so they go in {dest}.")
    return " ".join(out)
