import numpy as np

RGB_COLOURS = {
    "red": [255, 0, 0],
    "green": [0, 255, 0],
    "blue": [0, 0, 255],
    "white": [255, 255, 255],
    "black": [0, 0, 0],
    "yellow": [255, 255, 0],
    "cyan": [0, 255, 255],
    "magenta": [255, 0, 255],
    "gray": [128, 128, 128],
    "orange": [255, 165, 0],
    "purple": [128, 0, 128],
    "brown": [139, 69, 19],
    "pink": [255, 182, 193],
    "beige": [245, 245, 220],
    "maroon": [128, 0, 0],
    "olive": [128, 128, 0],
    "navy": [0, 0, 128],
    "lime": [50, 205, 50],
    "golden": [255, 223, 0],
    "teal": [0, 128, 128],
    "coral": [255, 127, 80],
    "salmon": [250, 128, 114],
    "turquoise": [64, 224, 208],
    "violet": [238, 130, 238],
    "platinum": [229, 228, 226],
    "ochre": [204, 119, 34],
    "burntsienna": [233, 116, 81],
    "chocolate": [210, 105, 30],
    "tan": [210, 180, 140],
    "ivory": [255, 255, 240],
    "goldenrod": [218, 165, 32],
    "orchid": [218, 112, 214],
    "honey": [238, 220, 130]
}

RGB_HAIR_COLOURS = {
    'midnight black': (9, 8, 6),
    'off black': (44, 34, 43),
    'strong dark brown': (58, 48, 36),
    'medium dark brown': (78, 67, 63),

    'chestnut brown': (106, 78, 66),
    'light chestnut brown': (106, 78, 66),
    'dark golden brown': (95, 72, 56),
    'light golden brown': (167, 133, 106),

    'dark honey blonde': (184, 151, 128),
    'bleached blonde': (220, 208, 186),
    'light ash blonde': (222, 288, 153),
    'light ash brown': (151, 121, 97),

    'lightest blonde': (230, 206, 168),
    'pale golden blonde': (229, 200, 168),
    'strawberry blonde': (165, 137, 70),
    'light auburn': (145, 85, 61),

    'dark auburn': (83, 61, 53),
    'darkest gray': (113, 99, 93),
    'medium gray': (183, 166, 158),
    'light gray': (214, 196, 194),

    'white blonde': (255, 24, 225),
    'platinum blonde': (202, 191, 177),
    'russet red': (145, 74, 67),
    'terra cotta': (181, 82, 57)}
