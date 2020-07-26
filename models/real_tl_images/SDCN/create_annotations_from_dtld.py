import argparse
import logging
import sys
import os
import yaml

import numpy as np
from dtld_parsing.calibration import CalibrationData

import cv2
from dtld_parsing.driveu_dataset import DriveuDatabase
import matplotlib.pyplot as plt

from PIL import Image

np.set_printoptions(suppress=True)

# Logging
logging.basicConfig(
    stream=sys.stdout,
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d %(levelname)s %(module)s - %(funcName)s: "
    "%(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

def parse_args():

    parser = argparse.ArgumentParser()
    parser.add_argument("--label_file", default="")
    parser.add_argument("--calib_dir", default="")
    parser.add_argument("--data_base_dir", default="")
    return parser.parse_args()

def main(args):
    # Load database
    database = DriveuDatabase(args.label_file)
    database.open(args.data_base_dir)

    # Load calibration
    calibration = CalibrationData()
    intrinsic_left = calibration.load_intrinsic_matrix(
        args.calib_dir + "/intrinsic_left.yml"
    )
    rectification_left = calibration.load_rectification_matrix(
        args.calib_dir + "/rectification_left.yml"
    )
    projection_left = calibration.load_projection_matrix(
        args.calib_dir + "/projection_left.yml"
    )
    extrinsic = calibration.load_extrinsic_matrix(
        args.calib_dir + "/extrinsic.yml"
    )
    distortion_left = calibration.load_distortion_matrix(
        args.calib_dir + "/distortion_left.yml"
    )

    # logging.info("Intrinsic Matrix:\n\n{}\n".format(intrinsic_left))
    # logging.info("Extrinsic Matrix:\n\n{}\n".format(extrinsic))
    # logging.info("Projection Matrix:\n\n{}\n".format(projection_left))
    # logging.info("Rectification Matrix:\n\n{}\n".format(rectification_left))
    # logging.info("Distortion Matrix:\n\n{}\n".format(distortion_left))

    output_dir = "DTLD_images"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    out = []
    for idx_d, driveu_img in enumerate(database.images):
        prefix = driveu_img.file_path.split('/')[-1].split('.')[0]
        status, img = driveu_img.get_image()
        image = Image.fromarray(img)

        annotations = []
        for i, o in enumerate(driveu_img.objects):
            area = o.width * o.height

            class_name = get_class_name(int(str(o.class_id)[-2]))

            bbox = {
                "class": class_name,
                "x_width": o.width,
                "y_height": o.height,
                "xmin": o.x,
                "ymin": o.y,
            }

            annotations.append(bbox)

        # if there is no annotation, break;
        if len(annotations) == 0:
            break

        output_path = "{}/{}.jpeg".format(output_dir, prefix)
        image.save(output_path)
        entry = {
            "annotations": annotations,
            "class": "image",
            "filename": output_path,
        }
        out.append(entry)

    with open('real_data_annotations_3.yaml', 'a') as outfile:
        yaml.dump(out, outfile, default_flow_style=False)

def get_class_name(class_id):
    class_name = "Unknown"

    if class_id == 1:
        class_name = "Red"
    elif class_id == 2 or class_id == 3:
        class_name = "Yellow"
    elif class_id == 4:
        class_name = "Green"

    return class_name

if __name__ == "__main__":
    main(parse_args())