import numpy as np
from lasr_vision_feature_extraction.categories_and_attributes import (
    CategoriesAndAttributes,
)


def _softmax(x: list[float]) -> list[float]:
    """Compute softmax values for each set of scores in x without using NumPy."""
    # First, compute e^x for each value in x
    exp_values = [_exp(val) for val in x]
    # Compute the sum of all e^x values
    sum_of_exp_values = sum(exp_values)
    # Now compute the softmax value for each original value in x
    softmax_values = [exp_val / sum_of_exp_values for exp_val in exp_values]
    return softmax_values


def _exp(x):
    """Compute e^x for a given x. A simple implementation of the exponential function."""
    return 2.718281828459045**x  # Using an approximation of Euler's number e


class ImageWithMasksAndAttributes:
    def __init__(
        self,
        image: np.ndarray,
        masks: dict[str, np.ndarray],
        attributes: dict[str, float],
        categories_and_attributes: CategoriesAndAttributes,
    ):
        self.image: np.ndarray = image
        self.masks: dict[str, np.ndarray] = masks
        self.attributes: dict[str, float] = attributes
        self.categories_and_attributes: CategoriesAndAttributes = (
            categories_and_attributes
        )

        self.plane_attribute_dict: dict[str, float] = {}
        for attribute in self.categories_and_attributes.plane_attributes:
            self.plane_attribute_dict[attribute] = self.attributes[attribute]

        self.selective_attribute_dict: dict[str, dict[str, float]] = {}
        for category in sorted(
            list(self.categories_and_attributes.selective_attributes.keys())
        ):
            self.selective_attribute_dict[category] = {}
            temp_list: list[float] = []
            for attribute in self.categories_and_attributes.selective_attributes[
                category
            ]:
                temp_list.append(self.attributes[attribute])
            softmax_list = _softmax(temp_list)
            for i, attribute in enumerate(
                self.categories_and_attributes.selective_attributes[category]
            ):
                self.selective_attribute_dict[category][attribute] = softmax_list[i]

    def describe(self) -> str:
        # abstract method
        pass


def _max_value_tuple(some_dict: dict[str, float]) -> tuple[str, float]:
    max_key = max(some_dict, key=some_dict.get)
    return max_key, some_dict[max_key]


class ImageOfPerson(ImageWithMasksAndAttributes):
    def __init__(
        self,
        image: np.ndarray,
        masks: dict[str, np.ndarray],
        attributes: dict[str, float],
        categories_and_attributes: CategoriesAndAttributes,
    ):
        super().__init__(image, masks, attributes, categories_and_attributes)

    @classmethod
    def from_parent_instance(
        cls, parent_instance: ImageWithMasksAndAttributes
    ) -> "ImageOfPerson":
        """
        Creates an instance of ImageOfPerson using the properties of an
        instance of ImageWithMasksAndAttributes.
        """
        return cls(
            image=parent_instance.image,
            masks=parent_instance.masks,
            attributes=parent_instance.attributes,
            categories_and_attributes=parent_instance.categories_and_attributes,
        )

    def describe(self) -> dict:
        male = (
            self.attributes["Male"]
            > self.categories_and_attributes.thresholds_pred["Male"],
            self.attributes["Male"],
        )
        has_hair = (
            self.attributes["hair"]
            > self.categories_and_attributes.thresholds_pred["hair"],
            self.attributes["hair"],
        )
        hair_colour = _max_value_tuple(self.selective_attribute_dict["hair_colour"])
        hair_shape = _max_value_tuple(self.selective_attribute_dict["hair_shape"])
        facial_hair = _max_value_tuple(self.selective_attribute_dict["facial_hair"])
        # bangs = (
        #     self.attributes['Bangs'] > self.categories_and_attributes.thresholds_pred['Bangs'],
        #     self.attributes['Bangs'])
        hat = (
            self.attributes["Wearing_Hat"]
            > self.categories_and_attributes.thresholds_pred["Wearing_Hat"],
            self.attributes["Wearing_Hat"],
        )
        glasses = (
            self.attributes["Eyeglasses"]
            > self.categories_and_attributes.thresholds_pred["Eyeglasses"],
            self.attributes["Eyeglasses"],
        )
        earrings = (
            self.attributes["Wearing_Earrings"]
            > self.categories_and_attributes.thresholds_pred["Wearing_Earrings"],
            self.attributes["Wearing_Earrings"],
        )
        necklace = (
            self.attributes["Wearing_Necklace"]
            > self.categories_and_attributes.thresholds_pred["Wearing_Necklace"],
            self.attributes["Wearing_Necklace"],
        )
        necktie = (
            self.attributes["Wearing_Necktie"]
            > self.categories_and_attributes.thresholds_pred["Wearing_Necktie"],
            self.attributes["Wearing_Necktie"],
        )

        description = "This customer has "
        hair_colour_str = "None"
        hair_shape_str = "None"
        if has_hair[0]:
            hair_shape_str = ""
            if hair_shape[0] == "Straight_Hair":
                hair_shape_str = "straight"
            elif hair_shape[0] == "Wavy_Hair":
                hair_shape_str = "wavy"
            if hair_colour[0] == "Black_Hair":
                description += "black %s hair, " % hair_shape_str
                hair_colour_str = "black"
            elif hair_colour[0] == "Blond_Hair":
                description += "blond %s hair, " % hair_shape_str
                hair_colour_str = "blond"
            elif hair_colour[0] == "Brown_Hair":
                description += "brown %s hair, " % hair_shape_str
                hair_colour_str = "brown"
            elif hair_colour[0] == "Gray_Hair":
                description += "gray %s hair, " % hair_shape_str
                hair_colour_str = "gray"

        if (
            male
        ):  # here 'male' is only used to determine whether it is confident to decide whether the person has beard
            if not facial_hair[0] == "No_Beard":
                description += "and has beard. "

        if hat[0] or glasses[0]:
            description += "I can also see this customer wearing "
            if hat[0] and glasses[0]:
                description += "a hat and a pair of glasses. "
            elif hat[0]:
                description += "a hat. "
            else:
                description += "a pair of glasses. "

        if earrings[0] or necklace[0] or necktie[0]:
            description += "This customer might also wear "
            wearables = []
            if earrings[0]:
                wearables.append("a pair of earrings")
            if necklace[0]:
                wearables.append("a necklace")
            if necktie[0]:
                wearables.append("a necktie")
            description += (
                ", ".join(wearables[:-2] + [" and ".join(wearables[-2:])]) + ". "
            )

        if description == "This customer has ":
            description = (
                "I didn't manage to get any attributes from this customer, I'm sorry."
            )

        result = {
            "attributes": {
                "has_hair": has_hair[0],
                "hair_colour": hair_colour_str,
                "hair_shape": hair_shape_str,
                # stop returning "male" label for now, might remove from neural network when training next time.
                # "male": male[0],
                "facial_hair": facial_hair[0] != "No_Beard",
                "hat": hat[0],
                "glasses": glasses[0],
                "earrings": earrings[0],
                "necklace": necklace[0],
                "necktie": necktie[0],
            },
            "description": description,
        }

        return result


class ImageOfCloth(ImageWithMasksAndAttributes):
    def __init__(
        self,
        image: np.ndarray,
        masks: dict[str, np.ndarray],
        attributes: dict[str, float],
        categories_and_attributes: CategoriesAndAttributes,
    ):
        super().__init__(image, masks, attributes, categories_and_attributes)

    @classmethod
    def from_parent_instance(
        cls, parent_instance: ImageWithMasksAndAttributes
    ) -> "ImageOfCloth":
        """
        Creates an instance of ImageOfCloth using the properties of an
        instance of ImageWithMasksAndAttributes.
        """
        return cls(
            image=parent_instance.image,
            masks=parent_instance.masks,
            attributes=parent_instance.attributes,
            categories_and_attributes=parent_instance.categories_and_attributes,
        )

    def describe(self) -> dict:
        top = (
            self.attributes["top"]
            > self.categories_and_attributes.thresholds_pred["top"]
        )
        down = (
            self.attributes["down"]
            > self.categories_and_attributes.thresholds_pred["down"]
        )
        dress = (
            self.attributes["dress"]
            > self.categories_and_attributes.thresholds_pred["dress"]
        )
        outwear = (
            self.attributes["outwear"]
            > self.categories_and_attributes.thresholds_pred["outwear"]
        )

        result = {
            # not in a loop for now, likely to add more logic combined with a classifier of more specific cloth classes.
            "attributes": {},
            "description": "this descrcription will be completed if we find out it is better to do it here.",
        }

        print(self.attributes)

        for attribute in [
            "short sleeve top",
            "long sleeve top",
            "short sleeve outwear",
            "long sleeve outwear",
            # 'shorts',
            # 'trousers',
            # 'skirt',
            "short sleeve dress",
            "long sleeve dress",
            "vest dress",
            "sling dress",
            "sleeveless top",
        ]:
            result["attributes"][attribute] = False

        if top:
            max_prob = 0.0
            max_attribute = "short sleeve top"
            for attribute in ["short sleeve top", "long sleeve top", "vest", "sling"]:
                if self.attributes[attribute] > max_prob:
                    max_prob = self.attributes[attribute]
                    max_attribute = attribute
            if max_attribute in ["vest", "sling"]:
                max_attribute = "sleeveless top"
            result["attributes"][max_attribute] = True

        # "down" is not included for now since it might not be necessary to mention
        # keep the comments here for showing all the cases.
        # if down:
        #     max_prob = 0.0
        #     max_attribute = "shorts"
        #     for attribute in ['shorts', 'trousers', 'skirt', ]:
        #         if self.attributes[attribute] > max_prob:
        #             max_prob = self.attributes[attribute]
        #             max_attribute = attribute
        #     result["attributes"][max_attribute] = True

        if outwear:
            max_prob = 0.0
            max_attribute = "short sleeve outwear"
            for attribute in [
                "short sleeve outwear",
                "long sleeve outwear",
            ]:
                if self.attributes[attribute] > max_prob:
                    max_prob = self.attributes[attribute]
                    max_attribute = attribute
            result["attributes"][max_attribute] = True

        if dress:
            max_prob = 0.0
            max_attribute = "short sleeve dress"
            for attribute in [
                "short sleeve dress",
                "long sleeve dress",
                "vest dress",
                "sling dress",
            ]:
                if self.attributes[attribute] > max_prob:
                    max_prob = self.attributes[attribute]
                    max_attribute = attribute
            result["attributes"][max_attribute] = True

        return result
