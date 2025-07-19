"""
State for recovering the speech transcribed via whisper (name and item) by using
the spelling and pronounciation of a word.
"""

import rospy
import smach
import string
import jellyfish as jf
from smach import UserData
from typing import List, Dict, Any


class SpeechRecovery(smach.State):
    def __init__(
        self,
        available_items,
        last_resort,
    ):
        """Recover the correct name and / or item by parsing the transcription.

        Args:
            guest_id (str): ID of the guest (identifying the guest)
            last_resort (bool): Whether the program must recover a name or item
            input_type (str, optional): The type of information to try and extract useful information
            (item or name)
        """

        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["customer_transcription", "order_str", "order"],
            output_keys=["order", "order_str"],
        )

        self._last_resort = last_resort

        # self._available_single_items = ["cola", "water", "milk", "fanta", "kuat"]
        # self._available_double_items = ["ice", "tea", "big", "coke"]
        # self._double_items_dict = {
        #     "ice": "ice tea",
        #     "tea": "ice tea",
        #     "big": "big coke",
        #     "coke": "big coke",
        # }
        # self._available_items = list(
        #     set(self._available_single_items).union(set(self._available_double_items))
        # )

        self._available_items = available_items

        # 1. Double-word items = items with a space
        self._available_double_items_whole = [
            d for d in self._available_items if " " in d
        ]
        self._available_double_items = [
            d.split() for d in self._available_items if len(d.split()) > 1
        ]
        if self._available_double_items:
            self._available_double_items = self._available_double_items[0]

        # 2. Single-word items = items with no space
        self._available_single_items = [
            d for d in self._available_items if " " not in d
        ]

        # 3. Mapping: Each word in a double item points to the double item
        self._double_items_dict = {
            w: d for d in self._available_double_items_whole for w in d.split()
        }

        self._excluded_words = ["can", "you", "get", "me", "i", "would", "like", "a"]

    def execute(self, userdata: UserData) -> str:
        """Attempt to recover the item or / and name from the LLM response or the transcription.

        Args:
            userdata (UserData): State machine userdata assumed to contain a key
            called "guest transcription" with the transcription of the guest's name or
            favourite item or both.

        Returns:
            str: state outcome. Updates the userdata with the parsed information (item or name), under
            the parameter "guest_data".
        """

        filtered_sentence = (
            userdata.customer_transcription.lower()
            .translate(str.maketrans("", "", string.punctuation))
            .strip()
        )
        sentence_split = filtered_sentence.split()
        sentence_list = list(set(sentence_split) - set(self._excluded_words))

        if not hasattr(userdata, "order") or userdata.order is None:
            pass
        else:
            print(self._available_items, userdata.order)
            self._available_items = list(
                set(self._available_items) - set(userdata.order)
            )
            self._available_double_items_whole = [
                d for d in self._available_items if " " in d
            ]
            self._available_double_items = [
                d.split() for d in self._available_items if len(d.split()) > 1
            ]
            if self._available_double_items:
                self._available_double_items = self._available_double_items[0]

            # 2. Single-word items = items with no space
            self._available_single_items = [
                d for d in self._available_items if " " not in d
            ]

            # 3. Mapping: Each word in a double item points to the double item
            self._double_items_dict = {
                w: d for d in self._available_double_items_whole for w in d.split()
            }

        if not sentence_list:
            return "failed"
        if isinstance(sentence_list, str):
            sentence_list = [sentence_list]
        print(sentence_list)

        final_item = self._handle_item(sentence_list, self._last_resort)
        if final_item != "unknown":
            # userdata.guest_data[self._guest_id]["item"] = final_item
            if not hasattr(userdata, "order") or userdata.order is None:
                userdata.order = []
            userdata.order.append(final_item)
            userdata.order_str = self._construct_string(userdata.order)
            print(f"Recovered item: {final_item} ")
            return "succeeded"
        else:
            return "failed"

    def _handle_name(self, sentence_list: List[str], last_resort: bool) -> str:
        """Attempt to recover the name in the transcription. First recover via spelling, then pronounciation.
        Enter last resort if necessary and recover the closest spelt name.

        Args:
            sentence_list (List[str]): Transcription split up as a list of strings.
            last_resort (bool): Whether the program must recover a name

        Returns:
            str: Recovered name. 'unknown' is returned if no name is recovered.
        """
        result = self._handle_similar_spelt(sentence_list, self._available_names, 1)
        if result != "unknown":
            print(f"name (spelt): {result}")
            return result
        else:
            result = self._handle_similar_sound(sentence_list, self._available_names, 0)
            print(f"name (sound): {result}")
        if not last_resort or result != "unknown":
            return result
        else:
            print("Last resort name")
            return self._handle_closest_spelt(sentence_list, self._available_names)

    def _handle_item(self, sentence_list: List[str], last_resort: bool) -> str:
        """Attempt to recover the item in the transcription. For phrases containing two words, try to infer the
        second word in the phrase. If this fails, attempt to recover the item via spelling, then pronounciation.
        Enter last resort if necessary and recover the closest spelt item.

        Args:
            sentence_list (List[str]): Transcription split up as a list of strings.
            last_resort (bool): Whether the program must recover a item

        Returns:
            str: Recovered item. 'unknown' is returned if no item is recovered.
        """
        result = self._infer_second_item(sentence_list)
        if result != "unknown":
            return result
        result = self._handle_similar_spelt(sentence_list, self._available_items, 1)
        if result != "unknown":
            print(f"item (spelt): {result}")
        else:
            result = self._handle_similar_sound(sentence_list, self._available_items, 0)
            print(f"item (sound): {result}")

        if result != "unknown":
            if result in self._available_single_items:
                print(f"final attempt item: {result}")
                return result
            else:
                sentence_list.append(result)
                return self._infer_second_item(sentence_list)
        else:
            if not last_resort:
                return "unknown"
            else:
                print("Last resort item")
                closest_spelt = self._handle_closest_spelt(
                    sentence_list, self._available_items
                )
                if closest_spelt in self._available_single_items:
                    print(f"final attempt during last resort item: {closest_spelt}")
                    return closest_spelt
                else:
                    sentence_list.append(closest_spelt)
                    return self._infer_second_item(sentence_list)

    def _handle_similar_spelt(
        self,
        sentence_list: List[str],
        available_words: List[str],
        distance_threshold: int,
    ) -> str:
        """Recover any word by spelling that has a similarity lower than the specified threshold
        when comparing each word in the sentence list to the list of available words.

        Args:
            sentence_list (List[str]): Transcription split up as a list of strings.
            available_words (List[str]): List of available words to compare to (items or names)
            distance_threshold (int): Similarity in terms of spelling distance required for a word to be recovered

        Returns:
            str: Recovered word. 'unknown' is returned if no word is recovered.
        """
        for input_word in sentence_list:
            for available_word in available_words:
                distance = self._get_damerau_levenshtein_distance(
                    input_word, available_word
                )
                if distance <= distance_threshold:
                    return available_word
        return "unknown"

    def _handle_similar_sound(
        self,
        sentence_list: List[str],
        available_words: List[str],
        distance_threshold: int,
    ) -> str:
        """Recover any word by pronounciation that has a similarity lower than the specified threshold
        when comparing each word in the sentence list to the list of available words.

        Args:
            sentence_list (List[str]): Transcription split up as a list of strings.
            available_words (List[str]): List of available words to compare to (items or names)
            distance_threshold (int): Similarity in terms of pronounciation distance required for a word to be recovered

        Returns:
            str: Recovered word or phrase. 'unknown' is returned if no word is recovered.
        """
        for input_word in sentence_list:
            for available_word in available_words:
                distance = self._get_levenshtein_soundex_distance(
                    input_word, available_word
                )
                if distance <= distance_threshold:
                    return available_word
        return "unknown"

    def _infer_second_item(self, sentence_list: List[str]) -> str:
        """Infer the second word of a two-worded item phrase and hence the entire phrase, if
        a word contained in any of the two-worded item phrases is detected.

        Args:
            sentence_list (List[str]): Transcription split up as a list of strings.

        Returns:
            str: Recovered item phrase. 'unknown' is returned if no item phrase is recovered.
        """
        for input_word in sentence_list:
            for available_word in self._available_double_items:
                if input_word == available_word:
                    print("INFERRED SECOND item")
                    return self._double_items_dict[input_word]
        return "unknown"

    def _handle_closest_spelt(
        self, sentence_list: List[str], choices: List[str]
    ) -> str:
        """Get the closest spelt word from the list of choices (items or names)
        in the sentence list (transcription).

        Args:
            sentence_list (List[str]): Transcription split up as a list of strings.
            choices (List[str]): List of choices to compare to (items or names)

        Returns:
            str: Recovered closest spelt word.
        """
        closest_distance = float("inf")
        closest_word = ""
        for input_word in sentence_list:
            for available_word in choices:
                distance = self._get_damerau_levenshtein_distance(
                    input_word, available_word
                )
                if distance < closest_distance:
                    closest_distance = distance
                    closest_word = available_word
        return closest_word

    def _get_damerau_levenshtein_distance(self, word_1: str, word_2: str) -> int:
        """Get the damerau-levenshtein distance between two words for the similarity in spelling.

        Args:
            word_1 (str): First word
            word_2 (str): Second word

        Returns:
            int: Damerau-levenshtein distance between the two words.
        """
        return jf.damerau_levenshtein_distance(word_1, word_2)

    def _get_levenshtein_soundex_distance(self, word_1: str, word_2: str) -> int:
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

    def _construct_string(self, transcription_items_list):
        order_string = ""
        for item in transcription_items_list:
            order_string += f" one {item},"
        return order_string.strip(",")
