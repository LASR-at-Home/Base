import argparse
from typing import Dict


def split_txt_file(input_file, output_file, num_splits):
    with open(input_file, "r", encoding="utf8") as src:
        lines = src.readlines()
    split_size = len(lines) // num_splits
    for i in range(num_splits):
        with open(f"{output_file}_chunk_{i+1}.txt", "w", encoding="utf8") as dest:
            dest.writelines(lines[i * split_size : (i + 1) * split_size])


def parse_args() -> Dict:
    parser = argparse.ArgumentParser(description="Split a txt file into chunks")
    parser.add_argument("input_file", type=str, help="Path to the input txt file")
    parser.add_argument("output_file", type=str, help="Path to the output txt file")
    parser.add_argument(
        "num_splits", type=int, help="Number of chunks to split the file into"
    )
    known, _ = parser.parse_known_args()
    return vars(known)


if __name__ == "__main__":
    args = parse_args()
    split_txt_file(args["input_file"], args["output_file"], args["num_splits"])
