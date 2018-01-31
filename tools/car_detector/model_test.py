import os
import numpy as np
from PIL import Image
from glob import glob
import tensorflow as tf
from matplotlib import pyplot as plt
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_s

IMAGE_TYPE = "sim"

ssd_real_model = "./models/real_inference_graph.pb"
ssd_sim_model = "./models/sim_inference_graph.pb"

if IMAGE_TYPE == "sim":
    ssd_model = ssd_sim_model
else:
    ssd_model = ssd_real_model

PATH_LABELS = 'label_map.pbtxt'

#The test image folder
#TEST_IMAGE_DIR = 'test_images_%s'%(IMAGE_TYPE)
TEST_IMAGE_DIR = 'camera_capture'

OUT_RESULT_DIR = './result/%s'%(IMAGE_TYPE)
OUT_RESULT_DIR = './result_camera_capture/'

label_map = label_map_util.load_labelmap(PATH_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map,\
              max_num_classes=5,use_display_name=True)

category_index = label_map_util.create_category_index(categories)
print(category_index)


def load_img_to_np(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(ssd_model, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')





TEST_IMAGE_PATHS = glob(os.path.join(TEST_IMAGE_DIR, '*.jpg'))
print("Length of test images:", len(TEST_IMAGE_PATHS))
IMAGE_SIZE = (12, 8)
with detection_graph.as_default():
    with tf.Session(graph=detection_graph) as sess:
        # Definite input and output Tensors for detection_graph
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        
        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')
        im_id = 0
        for image_path in TEST_IMAGE_PATHS:
            image = Image.open(image_path)
            # the array based representation of the image will be used later in order to prepare the
            # result image with boxes and labels on it.
            image_np = load_img_to_np(image)
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image_np, axis=0)

            
            # Actual detection.
            (boxes, scores, classes, num) = sess.run(
              [detection_boxes, detection_scores, detection_classes, num_detections],
              feed_dict={image_tensor: image_np_expanded})

        

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)
            
            # Visualization of the results of a detection.
            vis_s.visualize_boxes_and_labels_on_image_array(
                image_np, boxes, classes, scores,
                category_index,
                use_normalized_coordinates=True,
                line_thickness=6)
            
            plt.figure(figsize=IMAGE_SIZE)
            plt.imshow(image_np)
            plt.savefig(OUT_RESULT_DIR + '/%d.jpg'%(im_id))
            im_id += 1
            #plt.show()