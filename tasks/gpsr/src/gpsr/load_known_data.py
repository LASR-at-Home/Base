#!/usr/bin/env python

"""
GPSR regex parser (and command generator) requires a list of the following names:

    - objects
    - object categories plural
    - object categories singular
    - placeable locations
    - non-placeable locations
    - people
    - rooms
"""

import json
import os
from typing import List, Tuple, Dict


class GPSRDataLoader:
    """Loads the a priori known data for the GPSR task from the
    corresponding json files."""

    def __init__(self, data_dir: str = "../data/mock_data/") -> None:
        """Stores the data directory that contains the json files to load.

        Args:
            data_dir (str, optional): Path to the directory that contains the
            data json files. Defaults to "../data/mock_data/".
        """
        self._data_dir: str = data_dir

        # assumed constant file names
        self.LOCATIONS_FILE: str = "locations.json"
        self.OBJECTS_FILE: str = "objects.json"
        self.NAMES_FILE: str = "names.json"
        self.ROOMS_FILE: str = "rooms.json"

        if not self._validate_dir():
            raise ValueError(
                "The data directory does not contain all of the necessary json files."
            )

    def _validate_dir(self) -> bool:
        """Checks if all of the necessary json files are present in the
        data directory."""

        if not all(
            [
                self.LOCATIONS_FILE in os.listdir(self._data_dir),
                self.OBJECTS_FILE in os.listdir(self._data_dir),
                self.NAMES_FILE in os.listdir(self._data_dir),
                self.ROOMS_FILE in os.listdir(self._data_dir),
            ]
        ):
            return False
        return True

    def _load_names(self) -> List[str]:
        """Loads the names stored in the names json into a list of strings.

        Returns:
            List[str]: list of names found in the names.json file
        """

        with open(os.path.join(self._data_dir, self.NAMES_FILE), "r") as f:
            names = json.load(f)
        return names[
            "names"
        ]  # names is a dictionary with a single key "names" that contains a list of names

    def _load_rooms(self) -> List[str]:
        """Loads the rooms stored in the rooms json into a list of strings.

        Returns:
            List[str]: list of rooms found in the rooms.json file
        """

        with open(os.path.join(self._data_dir, self.ROOMS_FILE), "r") as f:
            rooms = json.load(f)
        return rooms[
            "rooms"
        ]  # rooms is a dictionary with a single key "rooms" that contains a list of rooms

    def _load_locations(self) -> Tuple[List[str], List[str]]:
        """Loads the locations into two lists of strings of location names:
        one list for placeable locations and the other for non-placeable.

        Returns:
            Tuple[List[str], List[str]]: placeable, non-placeable locations.
        """

        placeable_locations: List[str] = []
        non_placeable_locations: List[str] = []

        with open(os.path.join(self._data_dir, self.LOCATIONS_FILE), "r") as f:
            locations = json.load(f)

        for location, attributes in locations.items():
            if attributes["placeable"]:
                placeable_locations.append(location)
            non_placeable_locations.append(location)

        return placeable_locations, non_placeable_locations

    def _load_objects(self) -> Tuple[List[str], List[str], List[str]]:
        """Loads the objects into three lists of strings:
        one list for object names, one for the plural names of object categories,
        and one for the singular names of object categories.

        Returns:
            Tuple[List[str], List[str], List[str]]: objects, categories plural, categories singular
        """

        objects: List[str] = []
        categories_plural: List[str] = []
        categories_singular: List[str] = []

        with open(os.path.join(self._data_dir, self.OBJECTS_FILE), "r") as f:
            object_data = json.load(f)

        for category, category_data in object_data.items():
            categories_plural.append(category)
            categories_singular.append(category_data["singular"])
            objects.extend(category_data["items"])

        return objects, categories_plural, categories_singular

    def load_data(self) -> Dict[str, List[str]]:
        """Loads all of the known data for the GPSR task.

        Returns:
            Dict[str, List[str]]: dictionary containing the following
            keys and their corresponding lists of strings:
                - "names"
                - "rooms"
                - "placeable_locations"
                - "non_placeable_locations"
                - "objects"
                - "categories_plural"
                - "categories_singular"
        """

        names = self._load_names()
        rooms = self._load_rooms()
        placeable_locations, non_placeable_locations = self._load_locations()
        objects, categories_plural, categories_singular = self._load_objects()

        return {
            "names": names,
            "rooms": rooms,
            "placeable_locations": placeable_locations,
            "non_placeable_locations": non_placeable_locations,
            "objects": objects,
            "categories_plural": categories_plural,
            "categories_singular": categories_singular,
        }


if __name__ == "__main__":
    loader = GPSRDataLoader()
    data = loader.load_data()
    print(data)
