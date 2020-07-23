import argparse
import logging
import sys
import os

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

    sub_dir_names = ["Red", "Green", "Yellow", "None"]
    for sub_dir in sub_dir_names:
        dir_name = "{}/{}".format("images", sub_dir)
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)

    for idx_d, driveu_img in enumerate(database.images):
        status, img = driveu_img.get_image()
        image = Image.fromarray(img)
        for i, o in enumerate(driveu_img.objects):
            area = o.width * o.height

            if area > 250:
                class_id = int(str(o.class_id)[-2])
                unique_id = o.unique_id
                bbox = get_bbox(o.x, o.y, o.width, o.height, image)
                crop_image = image.crop(bbox)
                dir_name = get_dir_name(class_id)
                crop_image.save("{}/{}_{}_{}.jpg".format(
                    dir_name, unique_id, i, class_id), quality=100)
    
        if idx_d > 100:
            break

def get_dir_name(class_id):
    dir_name = "images/"
    if class_id == 1:
        dir_name += "Red"
    elif class_id == 2 or class_id == 3:
        dir_name += "Yellow"
    elif class_id == 4:
        dir_name += "Green"
    else:
        dir_name += "None"
    return dir_name

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