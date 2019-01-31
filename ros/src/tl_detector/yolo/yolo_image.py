import sys
from yolo import YOLO
from PIL import Image
import matplotlib.pyplot as plt
import keras.backend as K
import numpy as np
import ipdb
import os
import tqdm
import cv2
from yolo3.utils import letterbox_image

def get_traffic_lights_crop_with_YOLO(yolo, img):
    '''
    gets rgb image
    returns the bounding box coordinates of a traffic light (left, top, right, bottom)S
    '''
    
    img = img[200:800,:, :]
    image = Image.fromarray(img)
    boxed_image = letterbox_image(image, tuple(reversed(yolo.model_image_size)))
    image_data = np.array(boxed_image, dtype='float32')
    # ipdb.set_trace()
    image_data = image_data / 255.
    image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

    out_boxes, out_classes = yolo.sess.run(
        [yolo.boxes, yolo.classes],
        feed_dict={
            yolo.yolo_model.input: image_data,
            yolo.input_image_shape: [image.size[1], image.size[0]],
            K.learning_phase(): 0
        })

    for i, c in reversed(list(enumerate(out_classes))):
        if c == 9:
            box = out_boxes[i]
            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
            cropped_image = img[top:bottom, left:right, :]
            return cropped_image
    return None


# yolo = YOLO()
# input_folder = '/home/udacity-ros/data/udacity-simulator-data/carla-images/'
# outputs_folder = '/home/udacity-ros/data/udacity-simulator-data/carla-images-cropped2/'
# labels = ['0/', '1/', '2/']
# for label in labels[:1]:
#     label_input_folder = input_folder + label
#     label_output_folder = outputs_folder + label
#     imgs = os.listdir(label_input_folder)
#     failes = 0
#     for i in tqdm.tqdm(range(len(imgs))):
#     # for img in imgs:
#         img = imgs[i]
#         image = plt.imread(label_input_folder + img)
#         # ipdb.set_trace()
#         cropped_image = get_traffic_lights_crop_with_YOLO(yolo, image)
#         if cropped_image is not None:
#             plt.imsave(label_output_folder+img, cropped_image)
#             pass
#         else:
#             failes += 1
        
#     print("failed: %d times" % failes)
