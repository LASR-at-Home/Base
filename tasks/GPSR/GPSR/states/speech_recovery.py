"""
State for recovering the speech transcribed via whisper (name and drink) by using
the spelling and pronounciation of a word.
"""

import smach
import string
import jellyfish as jf
from smach import UserData
from typing import List

# TODO test this state


class SpeechRecovery(smach.State):
    def __init__(self, guest_id: int, last_resort: bool, input_type: str = ""):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_transcription", "guest_data"],
            output_keys=["guest_data", "guest_transcription"],
        )

        self._guest_id = guest_id
        self._last_resort = last_resort
        self._input_type = input_type
        self._available_names = [
            "sophie", "julia", "emma", "sara", "laura", "hayley", "susan",
            "fleur", "gabrielle", "robin", "john", "liam", "lucas",
            "william", "kevin", "jesse", "noah", "harrie", "peter",
        ]
        self._available_single_drinks = ["cola", "water", "milk", "fanta", "dubbelfris"]
        self._available_double_drinks = ["ice", "tea", "big", "coke"]
        self._double_drinks_dict = {
            "ice": "ice tea",
            "tea": "ice tea",
            "big": "big coke",
            "coke": "big coke",
        }
        self._available_drinks = list(
            set(self._available_single_drinks).union(set(self._available_double_drinks))
        )
        self._excluded_words = [
            "my", "name", "is", "and", "favourite", "drink", "you", "can", "call", "me",
        ]

    def execute(self, userdata: UserData) -> str:
        filtered_sentence = userdata.guest_transcription.lower().translate(
            str.maketrans("", "", string.punctuation)
        )
        sentence_split = filtered_sentence.split()
        sentence_list = list(set(sentence_split) - set(self._excluded_words))
        if not sentence_list:
            return "failed"

        if self._input_type == "name":
            final_name = self._handle_name(sentence_list, self._last_resort)
            if final_name != "unknown":
                userdata.guest_data[self._guest_id]["name"] = final_name
                return "succeeded"
            return "failed"

        if self._input_type == "drink":
            final_drink = self._handle_drink(sentence_list, self._last_resort)
            if final_drink != "unknown":
                userdata.guest_data[self._guest_id]["drink"] = final_drink
                return "succeeded"
            return "failed"

        if userdata.guest_data[self._guest_id]["name"] == "unknown":
            userdata.guest_data[self._guest_id]["name"] = self._handle_name(
                sentence_list, self._last_resort
            )
        if userdata.guest_data[self._guest_id]["drink"] == "unknown":
            userdata.guest_data[self._guest_id]["drink"] = self._handle_drink(
                sentence_list, self._last_resort
            )
        if (
            userdata.guest_data[self._guest_id]["name"] == "unknown"
            or userdata.guest_data[self._guest_id]["drink"] == "unknown"
        ):
            return "failed"
        return "succeeded"

    def _handle_name(self, sentence_list: List[str], last_resort: bool) -> str:
        result = self._handle_similar_spelt(sentence_list, self._available_names, 1)
        if result != "unknown":
            return result
        result = self._handle_similar_sound(sentence_list, self._available_names, 0)
        if not last_resort or result != "unknown":
            return result
        return self._handle_closest_spelt(sentence_list, self._available_names)

    def _handle_drink(self, sentence_list: List[str], last_resort: bool) -> str:
        result = self._infer_second_drink(sentence_list)
        if result != "unknown":
            return result
        result = self._handle_similar_spelt(sentence_list, self._available_drinks, 1)
        if result == "unknown":
            result = self._handle_similar_sound(sentence_list, self._available_drinks, 0)
        if result != "unknown":
            if result in self._available_single_drinks:
                return result
            sentence_list.append(result)
            return self._infer_second_drink(sentence_list)
        if not last_resort:
            return "unknown"
        if self._recover_dubbelfris(sentence_list):
            return "dubbelfris"
        closest_spelt = self._handle_closest_spelt(sentence_list, self._available_drinks)
        if closest_spelt in self._available_single_drinks:
            return closest_spelt
        sentence_list.append(closest_spelt)
        return self._infer_second_drink(sentence_list)

    def _handle_similar_spelt(
        self, sentence_list: List[str], available_words: List[str], distance_threshold: int
    ) -> str:
        for input_word in sentence_list:
            for available_word in available_words:
                if self._get_damerau_levenshtein_distance(input_word, available_word) <= distance_threshold:
                    return available_word
        return "unknown"

    def _handle_similar_sound(
        self, sentence_list: List[str], available_words: List[str], distance_threshold: int
    ) -> str:
        for input_word in sentence_list:
            for available_word in available_words:
                if self._get_levenshtein_soundex_distance(input_word, available_word) <= distance_threshold:
                    return available_word
        return "unknown"

    def _infer_second_drink(self, sentence_list: List[str]) -> str:
        for input_word in sentence_list:
            for available_word in self._available_double_drinks:
                if input_word == available_word:
                    return self._double_drinks_dict[input_word]
        return "unknown"

    def _handle_closest_spelt(self, sentence_list: List[str], choices: List[str]) -> str:
        closest_distance = float("inf")
        closest_word = None
        for input_word in sentence_list:
            for available_word in choices:
                distance = self._get_damerau_levenshtein_distance(input_word, available_word)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_word = available_word
        return closest_word

    def _recover_dubbelfris(self, sentence_list: List[str]) -> bool:
        for word in sentence_list:
            if self._get_levenshtein_soundex_distance("dubbelfris", word) < 3:
                return True
        return False

    def _get_damerau_levenshtein_distance(self, word_1: str, word_2: str) -> int:
        return jf.damerau_levenshtein_distance(word_1, word_2)

    def _get_levenshtein_soundex_distance(self, word_1: str, word_2: str) -> int:
        return jf.levenshtein_distance(jf.soundex(word_1), jf.soundex(word_2))
