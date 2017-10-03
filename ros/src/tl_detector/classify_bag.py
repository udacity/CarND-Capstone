import cv2
import numpy as np
import os
import rosbag
import video
from light_classification.tl_classifier import TLClassifier
from styx_msgs.msg import TrafficLight

class ImageMsg:
    def __init__(self, msg):
        self.header = msg.header
        self.height = msg.height
        self.width = msg.width
        img = np.fromstring(msg.data, dtype=np.uint8)
        img = img.reshape(msg.height, msg.width)
        self.raw = img

        # The test bags provided by Udacity contain images in BAYER_GB.
        self.bgr = cv2.cvtColor(img, cv2.COLOR_BAYER_GB2BGR)

def save_np_image(npimg, fullpath, bbox = None):
    if bbox is not None:
        cv2.rectangle(npimg, bbox[0], bbox[1], color = (255, 0, 0))
    cv2.imwrite(fullpath, npimg)

def read_images(bag_file):
    bag = rosbag.Bag(bag_file, "r")
    messages = bag.read_messages(topics=["/image_raw"])
    num_images = bag.get_message_count(topic_filters=["/image_raw"])

    for i in range(num_images):
        topic, msg, t  = messages.next()
        yield ImageMsg(msg)

    bag.close()

def friendly_name(pred):
    pred_name_dict = {TrafficLight.UNKNOWN: "Unknown",
                      TrafficLight.RED: "Red",
                      TrafficLight.GREEN: "Green"}

    return pred_name_dict[pred]

def embed_in_larger_image(img, scale):
    assert(scale >= 1.0)
    src_shape = img.shape
    dst_shape = (int(img.shape[0] * scale), int(img.shape[1] * scale), img.shape[2])
    print(src_shape, dst_shape)
    row = (dst_shape[0] / 2) - (src_shape[0] / 2)
    col = (dst_shape[1] / 2) - (src_shape[1] / 2)
    canvas = np.zeros(dst_shape, img.dtype)
    np.copyto(canvas[row : dst_shape[0] - row, col : dst_shape[1] - col], img)
    return canvas

def try_resize(bag_file):
    imgs = read_images(bag_file)
    larger = embed_in_larger_image(imgs[0].raw, 1.5)
    save_np_image(larger, 'larger.png')

def try_color_space(bag_file):
    imgs = read_images(bag_file)

    colors = [cv2.COLOR_BAYER_BG2BGR,
              cv2.COLOR_BAYER_GB2BGR,
              cv2.COLOR_BAYER_RG2BGR,
              cv2.COLOR_BAYER_GR2BGR,
              cv2.COLOR_BAYER_BG2RGB,
              cv2.COLOR_BAYER_GB2RGB,
              cv2.COLOR_BAYER_RG2RGB,
              cv2.COLOR_BAYER_GR2RGB]

    i = 0
    for c in colors:
        img = cv2.cvtColor(imgs[0].raw, c)
        save_np_image(img, 'color' + str(i) + '.png')
        i += 1

def preds_to_string(preds):
    result = '['
    for pred in preds:
        result += '{0:.2f} '.format(pred)
    result += ']'
    return result

def classify_bag(classifier, bag_file, run_name):
    print('bag_file:', bag_file)
    imgs = read_images(bag_file)

    # save_np_image(imgs[0].bgr, 'test.png')

    video_maker = video.VideoMaker('./out')
    for img in imgs:
        (pred, preds) = classifier.get_classification_detailed(img.bgr)
        # print('pred: ', pred, friendly_name(pred))
        labeled = img.bgr
        labeled = cv2.putText(labeled, friendly_name(pred), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 3, (255,255,255), 5)
        labeled = cv2.putText(labeled, preds_to_string(preds), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 3, (255,255,255), 5)
        video_maker.add_image(labeled)

    video_maker.make_video(os.path.basename(bag_file) + run_name + '.mp4')

if __name__ == '__main__':
    # try_resize('/home/eljefec/data/traffic_light_bag_files/just_traffic_light.bag')

    classifier = TLClassifier(sim = False)

    run_name = '.candidate'

    classify_bag(classifier, '/home/eljefec/data/traffic_light_bag_files/udacity_successful_light_detection.bag', run_name)

    classify_bag(classifier, '/home/eljefec/data/traffic_light_bag_files/just_traffic_light.bag', run_name)

    classify_bag(classifier, '/home/eljefec/data/traffic_light_bag_files/loop_with_traffic_light.bag', run_name)
