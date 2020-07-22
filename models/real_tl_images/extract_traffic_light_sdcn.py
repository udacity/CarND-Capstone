import argparse
import logging
import sys
import os

import io
import yaml
import numpy as np

from PIL import Image


def parse_args():

    parser = argparse.ArgumentParser()
    parser.add_argument("--input_yaml", default="")
    return parser.parse_args()

def main(args):

    sub_dir_names = ["Red", "Green", "Yellow", "None"]
    for sub_dir in sub_dir_names:
        dir_name = "{}/{}".format("images", sub_dir)
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)

    examples = yaml.load(open(args.input_yaml, 'rb').read())
    for i in range(len(examples)):
        examples[i]['filename'] = os.path.abspath(
            os.path.join(
                os.path.dirname(args.input_yaml), examples[i]['filename']))
    L = []
    for ex in examples:
        file_name = ex['filename']
        image = Image.open(file_name)

        x = ex['annotations'][0]['xmin']
        y = ex['annotations'][0]['ymin']
        width = ex['annotations'][0]['x_width']
        height = ex['annotations'][0]['y_height']
        bbox = get_bbox(x, y, width, height, image)

        crop_image = image.crop(bbox)
        
        class_name = ex['annotations'][0]['class']
        dir_name = "images/{}".format(class_name)
        unique_id = file_name.split('/')[-1].split('.')[0]
        class_id = get_class_id(class_name)
        crop_image.save("{}/{}_0_{}.jpg".format(dir_name, unique_id, class_id))

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