import argparse
import logging
import sys
import os

import io
import yaml
import numpy as np

import PIL
from PIL import Image


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_yaml", default="")
    parser.add_argument("--output_dir", default="")
    return parser.parse_args()

def main(args):

    sub_dir_names = ["Red", "Green", "Yellow", "None"]
    for sub_dir in sub_dir_names:
        dir_name = "{}/{}".format(args.output_dir, sub_dir)
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)

    with open(args.input_yaml) as file_name:
        examples = yaml.load(file_name, Loader=yaml.FullLoader)

    for i in range(len(examples)):
        examples[i]['filename'] = os.path.abspath(
            os.path.join(
                os.path.dirname(args.input_yaml), examples[i]['filename']))
    L = []
    # image
    for example in examples:
        file_name = example['filename']
        try:
            image = Image.open(file_name)
        except PIL.UnidentifiedImageError:
            continue

        # bboxes:
        for i, annotation in enumerate(example["annotations"]):
            x = annotation['xmin']
            y = annotation['ymin']
            width = annotation['x_width']
            height = annotation['y_height']

            if width * height < 400:
                continue
            
            if width * 2 > height:
                continue

            bbox = get_bbox(x, y, width, height, image)
            crop_image = image.crop(bbox)

            class_name = annotation['class']
            dir_name = "{}/{}".format(args.output_dir, class_name)
            unique_id = file_name.split('/')[-1].split('.')[0]
            class_id = get_class_id(class_name)
            output_path = "{}/{}_{}_{}.jpg".format(dir_name, unique_id, i, class_id)
            try:
                crop_image.save(output_path)
            except SystemError:
                if os.path.exists(output_path):
                    os.remove(output_path)
                continue

def get_class_id(class_name):
    if class_name == "Red":
        return 1
    elif class_name == "Yellow":
        return 2
    elif class_name == "Green":
        return 3
    else:
        return 0

def get_bbox(x, y, width, height, image):
    x_buffer = width // 10
    y_buffer = height // 10
    l = max(0, x - x_buffer)
    t = max(0, y - y_buffer)
    r = min(image.size[0], x + width + x_buffer)
    b = min(image.size[1], y + height + y_buffer)
    return (l, t, r, b)

if __name__ == "__main__":
    main(parse_args())