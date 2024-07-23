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


class DeepFashion2GeneralizedCategoriesAndAttributes(CategoriesAndAttributes):
    mask_categories = [
        "short sleeve top",
        "long sleeve top",
        "short sleeve outwear",
        "long sleeve outwear",
        "vest",
        "sling",
        "shorts",
        "trousers",
        "skirt",
        "short sleeve dress",
        "long sleeve dress",
        "vest dress",
        "sling dress",
    ]
    merged_categories = {
        "top": [
            "short sleeve top",
            "long sleeve top",
            "vest",
            "sling",
        ],
        "down": [
            "shorts",
            "trousers",
            "skirt",
        ],
        "outwear": [
            "short sleeve outwear",
            "long sleeve outwear",
        ],
        "dress": [
            "short sleeve dress",
            "long sleeve dress",
            "vest dress",
            "sling dress",
        ],
    }
    mask_labels = [
        "top",
        "down",
        "outwear",
        "dress",
    ]
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
        "short sleeve top",
        "long sleeve top",
        "short sleeve outwear",
        "long sleeve outwear",
        "vest",
        "sling",
        "shorts",
        "trousers",
        "skirt",
        "short sleeve dress",
        "long sleeve dress",
        "vest dress",
        "sling dress",
    ]

    thresholds_mask: dict[str, float] = {}
    thresholds_pred: dict[str, float] = {}

    # set default thresholds:
    for key in sorted(merged_categories.keys()):
        thresholds_mask[key] = 0.5
    for key in attributes + mask_labels:
        thresholds_pred[key] = 0.5
    pass
