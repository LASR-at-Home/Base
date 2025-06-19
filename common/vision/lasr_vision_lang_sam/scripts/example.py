#!/usr/bin/env python3.10
# Get python version

from lang_sam import LangSAM
from PIL import Image


def main():
    lang_sam = LangSAM()
    images = [Image.open("/home/mattbarker/dev/lang-segment-anything/assets/car.jpeg")]
    prompts = ["wheel"]
    results = lang_sam.predict(images, prompts, box_threshold=0.3, text_threshold=0.25)

    for idx, result in enumerate(results):
        print(result)
        print(result.keys())


if __name__ == "__main__":
    main()
