import rospy
import smach
import string
import jellyfish as jf
from smach import UserData
from typing import List, Dict, Any


class SpeechRecovery(smach.State):
    def __init__(
        self,
        guest_id: int,
        last_resort: bool,
        input_type: str = "",
    ):
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
            "sophie",
            "julia",
            "emma",
            "sara",
            "laura",
            "hayley",
            "susan",
            "fleur",
            "gabrielle",
            "robin",
            "john",
            "liam",
            "lucas",
            "william",
            "kevin",
            "jesse",
            "noah",
            "harrie",
            "peter",
        ]
        self._available_single_drinks = [
            "cola",
            "water",
            "milk",
            "fanta",
            "dubbelfris",
        ]
        self._available_double_drinks = [
            "ice",
            "tea",
            "big",
            "coke",
        ]
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

    def execute(self, userdata: UserData) -> str:
        filtered_sentence = userdata.guest_transcription.lower().translate(
            str.maketrans("", "", string.punctuation)
        )
        sentence_split = filtered_sentence.split()
        sentence_list = list(set(sentence_split) - set(self._excluded_words))
        if not sentence_list:
            return "failed"
        print(sentence_split)
        if self._input_type == "name":
            final_name = self._handle_name(sentence_list, self._last_resort)
            if final_name != "unknown":
                userdata.guest_data[self._guest_id]["name"] = final_name
                print(f"Recovered name: {final_name} ")
                return "succeeded"
            else:
                return "failed"
        elif self._input_type == "drink":
            final_drink = self._handle_drink(sentence_list, self._last_resort)
            if final_drink != "unknown":
                userdata.guest_data[self._guest_id]["drink"] = final_drink
                print(f"Recovered drink: {final_drink} ")
                return "succeeded"
            else:
                return "failed"
        else:
            if userdata.guest_data[self._guest_id]["name"] == "unknown":
                final_name = self._handle_name(sentence_list, self._last_resort)
                userdata.guest_data[self._guest_id]["name"] = final_name
                print(f"Recovered name: {final_name} ")
            if userdata.guest_data[self._guest_id]["drink"] == "unknown":
                final_drink = self._handle_drink(sentence_list, self._last_resort)
                userdata.guest_data[self._guest_id]["drink"] = final_drink
                print(f"Recovered drink: {final_drink} ")
            if (
                userdata.guest_data[self._guest_id]["name"] == "unknown"
                or userdata.guest_data[self._guest_id]["drink"] == "unknown"
            ):
                return "failed"
            else:
                return "succeeded"

    def _handle_name(self, sentence_list, last_resort):
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

    def _handle_drink(self, sentence_list, last_resort):
        result = self._infer_second_drink(sentence_list)
        if result != "unknown":
            return result
        result = self._handle_similar_spelt(sentence_list, self._available_drinks, 1)
        if result != "unknown":
            print(f"drink (spelt): {result}")
        else:
            result = self._handle_similar_sound(
                sentence_list, self._available_drinks, 0
            )
            print(f"drink (sound): {result}")

        if result != "unknown":
            if result in self._available_single_drinks:
                print(f"final attempt drink: {result}")
                return result
            else:
                sentence_list.append(result)
                return self._infer_second_drink(sentence_list)
        else:
            if not last_resort:
                return "unknown"
            else:
                print("Last resort drink")
                for word in sentence_list:
                    print(word)
                    print(self._get_levenshtein_soundex_distance("dubbelfris", word))
                    if self._get_levenshtein_soundex_distance("dubbelfris", word) < 3:
                        return "dubbelfris"
                closest_spelt = self._handle_closest_spelt(
                    sentence_list, self._available_drinks
                )
                if closest_spelt in self._available_single_drinks:
                    print(f"final attempt during last resort drink: {closest_spelt}")
                    return closest_spelt
                else:
                    sentence_list.append(closest_spelt)
                    return self._infer_second_drink(sentence_list)

    def _handle_similar_spelt(self, sentence_list, available_words, distance_threshold):
        for input_word in sentence_list:
            for available_word in available_words:
                distance = self._get_damerau_levenshtein_distance(
                    input_word, available_word
                )
                if distance <= distance_threshold:
                    return available_word
        return "unknown"

    def _handle_similar_sound(self, sentence_list, available_words, distance_threshold):
        for input_word in sentence_list:
            for available_word in available_words:
                distance = self._get_levenshtein_soundex_distance(
                    input_word, available_word
                )
                if distance <= distance_threshold:
                    print(input_word, available_word)
                    return available_word
        return "unknown"

    def _infer_second_drink(self, sentence_list):
        for input_word in sentence_list:
            for available_word in self._available_double_drinks:
                if input_word == available_word:
                    return self._double_drinks_dict[input_word]
        return "unknown"

    def _handle_closest_spelt(self, sentence_list, choices):
        closest_distance = float("inf")
        closest_word = None
        for input_word in sentence_list:
            for available_word in choices:
                distance = self._get_damerau_levenshtein_distance(
                    input_word, available_word
                )
                if distance < closest_distance:
                    closest_distance = distance
                    closest_word = available_word
        return closest_word

    def _get_damerau_levenshtein_distance(self, word_1, word_2):
        return jf.damerau_levenshtein_distance(word_1, word_2)

    def _get_levenshtein_soundex_distance(self, word_1, word_2):
        soundex_word_1 = jf.soundex(word_1)
        soundex_word_2 = jf.soundex(word_2)
        return jf.levenshtein_distance(soundex_word_1, soundex_word_2)
