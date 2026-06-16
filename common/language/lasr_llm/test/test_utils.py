# Copyright 2026 King's College London
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import importlib.util
from pathlib import Path


UTILS_PATH = Path(__file__).resolve().parents[1] / "lasr_llm" / "utils.py"
SPEC = importlib.util.spec_from_file_location("lasr_llm_utils", UTILS_PATH)
UTILS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(UTILS)
parse_llm_output_to_dict = UTILS.parse_llm_output_to_dict


def test_parse_llm_output_accepts_equals_separator():
    parsed = parse_llm_output_to_dict(
        "favourite drink = pepsi",
        ["Favourite drink"],
    )

    assert parsed == {"Favourite drink": "pepsi"}


def test_parse_llm_output_accepts_case_and_spelling_variants():
    parsed = parse_llm_output_to_dict(
        "NAME: Alice\nfavorite_drink: Orange juice\ninterest: robotics",
        ["Name", "Favourite drink", "Interests"],
    )

    assert parsed == {
        "Name": "Alice",
        "Favourite drink": "Orange juice",
        "Interests": "robotics",
    }


def test_parse_llm_output_ignores_requested_fields_echo():
    parsed = parse_llm_output_to_dict(
        "fields: name, favourite drink",
        ["Name", "Favourite drink"],
    )

    assert parsed == {"Name": None, "Favourite drink": None}


def test_parse_llm_output_accepts_inline_fields():
    parsed = parse_llm_output_to_dict(
        "Name: Alice, Favourite drink = pepsi",
        ["Name", "Favourite drink"],
    )

    assert parsed == {"Name": "Alice", "Favourite drink": "pepsi"}
