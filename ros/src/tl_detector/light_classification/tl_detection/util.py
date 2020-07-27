import os
import tarfile
#import wget
from PIL import ImageDraw

def prepare_tensorflow_object_detection_api(path,
    model="ssd_inception_v2_coco_2017_11_17"):

     dl_root = "http://download.tensorflow.org/models/object_detection"
     dl_name = "{}.tar.gz".format(model)
     dl_path = os.path.join(dl_root, dl_name)

     model = os.path.join(path, model)
     dl_name = os.path.join(path, dl_name)

     if not os.path.exists(model):
         if not os.path.exists(dl_name):
             wget.download(dl_path, path)
         with tarfile.open(dl_name) as f:
             f.extractall(path)
         os.remove(dl_name)

def draw_boxes(image, boxes, thickness=4):
     """Draw bounding boxes on the image"""
     draw = ImageDraw.Draw(image)
     for i in range(len(boxes)):
         bot, left, top, right = boxes[i, ...]
         draw.line([(left, top), (left, bot), (right, bot), (right, top), (left, top)], width=thickness, fill='red')
