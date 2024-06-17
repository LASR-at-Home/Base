class CategoriesAndAttributes:
    mask_categories: list[str] = []
    merged_categories: dict[str, list[str]] = {}
    mask_labels: list[str] = []
    selective_attributes: dict[str, list[str]] = {}
    plane_attributes: list[str] = []
    avoided_attributes: list[str] = []
    attributes: list[str] = []
    thresholds_mask: dict[str, float] = {}
    thresholds_pred: dict[str, float] = {}


class CelebAMaskHQCategoriesAndAttributes(CategoriesAndAttributes):
    mask_categories = [
        "cloth",
        "r_ear",
        "hair",
        "l_brow",
        "l_eye",
        "l_lip",
        "mouth",
        "neck",
        "nose",
        "r_brow",
        "r_ear",
        "r_eye",
        "skin",
        "u_lip",
        "hat",
        "l_ear",
        "neck_l",
        "eye_g",
    ]
    merged_categories = {
        "ear": [
            "l_ear",
            "r_ear",
        ],
        "brow": [
            "l_brow",
            "r_brow",
        ],
        "eye": [
            "l_eye",
            "r_eye",
        ],
        "mouth": [
            "l_lip",
            "u_lip",
            "mouth",
        ],
    }
    _categories_to_merge = []
    for key in sorted(list(merged_categories.keys())):
        for cat in merged_categories[key]:
            _categories_to_merge.append(cat)
    for key in mask_categories:
        if key not in _categories_to_merge:
            merged_categories[key] = [key]
    mask_labels = ["hair"]
    selective_attributes = {
        "facial_hair": [
            "5_o_Clock_Shadow",
            "Goatee",
            "Mustache",
            "No_Beard",
            "Sideburns",
        ],
        "hair_colour": [
            "Black_Hair",
            "Blond_Hair",
            "Brown_Hair",
            "Gray_Hair",
        ],
        "hair_shape": [
            "Straight_Hair",
            "Wavy_Hair",
        ],
    }
    plane_attributes = [
        "Bangs",
        "Eyeglasses",
        "Wearing_Earrings",
        "Wearing_Hat",
        "Wearing_Necklace",
        "Wearing_Necktie",
        "Male",
    ]
    avoided_attributes = [
        "Arched_Eyebrows",
        "Bags_Under_Eyes",
        "Big_Lips",
        "Big_Nose",
        "Bushy_Eyebrows",
        "Chubby",
        "Double_Chin",
        "High_Cheekbones",
        "Narrow_Eyes",
        "Oval_Face",
        "Pointy_Nose",
        "Receding_Hairline",
        "Rosy_Cheeks",
        "Heavy_Makeup",
        "Wearing_Lipstick",
        "Attractive",
        "Blurry",
        "Mouth_Slightly_Open",
        "Pale_Skin",
        "Smiling",
        "Young",
    ]
    attributes = [
        "5_o_Clock_Shadow",
        "Arched_Eyebrows",
        "Attractive",
        "Bags_Under_Eyes",
        "Bald",
        "Bangs",
        "Big_Lips",
        "Big_Nose",
        "Black_Hair",
        "Blond_Hair",
        "Blurry",
        "Brown_Hair",
        "Bushy_Eyebrows",
        "Chubby",
        "Double_Chin",
        "Eyeglasses",
        "Goatee",
        "Gray_Hair",
        "Heavy_Makeup",
        "High_Cheekbones",
        "Male",
        "Mouth_Slightly_Open",
        "Mustache",
        "Narrow_Eyes",
        "No_Beard",
        "Oval_Face",
        "Pale_Skin",
        "Pointy_Nose",
        "Receding_Hairline",
        "Rosy_Cheeks",
        "Sideburns",
        "Smiling",
        "Straight_Hair",
        "Wavy_Hair",
        "Wearing_Earrings",
        "Wearing_Hat",
        "Wearing_Lipstick",
        "Wearing_Necklace",
        "Wearing_Necktie",
        "Young",
    ]

    thresholds_mask: dict[str, float] = {}
    thresholds_pred: dict[str, float] = {}

    # set default thresholds:
    for key in sorted(merged_categories.keys()):
        thresholds_mask[key] = 0.5
    for key in attributes + mask_labels:
        if key not in avoided_attributes:
            thresholds_pred[key] = 0.5

    # set specific thresholds:
    thresholds_mask["eye_g"] = 0.5
    thresholds_pred["Eyeglasses"] = 0.5
    thresholds_pred["Wearing_Earrings"] = 0.5
    thresholds_pred["Wearing_Necklace"] = 0.5
    thresholds_pred["Wearing_Necktie"] = 0.5


class DeepFashion2GeneralizedCategoriesAndAttributes(CategoriesAndAttributes):
    mask_categories = [
        'short sleeve top', 'long sleeve top', 'short sleeve outwear',
        'long sleeve outwear', 'vest', 'sling', 'shorts',
        'trousers', 'skirt', 'short sleeve dress',
        'long sleeve dress', 'vest dress', 'sling dress'
    ]
    merged_categories = {
        'top': ['short sleeve top', 'long sleeve top', 'vest', 'sling', ],
        'down': ['shorts', 'trousers', 'skirt', ],
        'outwear': ['short sleeve outwear', 'long sleeve outwear', ],
        'dress': ['short sleeve dress', 'long sleeve dress', 'vest dress', 'sling dress', ],
    }
    mask_labels = ['top', 'down', 'outwear', 'dress', ]
    _categories_to_merge = []
    for key in sorted(list(merged_categories.keys())):
        for cat in merged_categories[key]:
            _categories_to_merge.append(cat)
    for key in mask_categories:
        if key not in _categories_to_merge:
            merged_categories[key] = [key]
    selective_attributes = {}
    plane_attributes = []
    avoided_attributes = []
    attributes = [
        'short sleeve top', 'long sleeve top', 'short sleeve outwear',
        'long sleeve outwear', 'vest', 'sling', 'shorts',
        'trousers', 'skirt', 'short sleeve dress',
        'long sleeve dress', 'vest dress', 'sling dress'
    ]

    thresholds_mask: dict[str, float] = {}
    thresholds_pred: dict[str, float] = {}

    # set default thresholds:
    for key in sorted(merged_categories.keys()):
        thresholds_mask[key] = 0.5
    for key in attributes + mask_labels:
        thresholds_pred[key] = 0.5
    pass
