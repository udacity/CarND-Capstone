import os
import base64
import glob
import json
import yaml

import matplotlib.pyplot as plt
from PIL import Image


def get_entry(json_file):
    data = json.load(open(json_file))
    image_path = data['imagePath']
    points = data['shapes'][0]['points']
    label = int(data['shapes'][0]['label'])
    parent_dir = '/'.join(json_file.split('/')[:-1])

    x_min = points[0][0]
    y_min = points[0][1]
    x_width = points[1][0] - points[0][0]
    y_height = points[1][1] - points[0][1]

    class_name = get_class_name(label)
    filename = "{}/{}".format(parent_dir, image_path)

    bbox = {
        "class": class_name,
        "x_width": x_width,
        "y_height": y_height,
        "xmin": x_min,
        "ymin": y_min
    }

    entry = {
        "annotations": [bbox],
        "class": "image",
        "filename": filename,
    }

    return entry

def get_class_name(class_id):
    class_name = "Unknown"
    if class_id == 1:
        class_name = "Red"
    elif class_id == 2:
        class_name = "Yellow"
    elif class_id == 3:
        class_name = "Green"
    return class_name

def main():
    out = []

    for filename in glob.glob("new_samples/*.json"):
        try:
            Image.open(filename.replace(".json", ".jpg"))
            out.append(get_entry(filename))
        except OSError:
            continue

    with open('real_data_annotations_2.yaml', 'w') as outfile:
        yaml.dump(out, outfile, default_flow_style=False)


if __name__ == "__main__":
    main()