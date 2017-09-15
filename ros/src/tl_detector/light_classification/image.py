from __future__ import print_function
import cv2
import numpy as np
import os
import rosbag
from consts import IMAGE_WIDTH, IMAGE_HEIGHT
from os import listdir
from os.path import isdir, isfile, join
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
        self.rgb = cv2.cvtColor(img, cv2.COLOR_BAYER_GB2RGB)

def save_np_image(npimg, fullpath, bbox = None):
    if bbox is not None:
        cv2.rectangle(npimg, bbox[0], bbox[1], color = (255, 0, 0))
    cv2.imwrite(fullpath, npimg)

def read_images(bag_file):
    bag = rosbag.Bag(bag_file, "r")
    messages = bag.read_messages(topics=["/image_raw"])
    num_images = bag.get_message_count(topic_filters=["/image_raw"])

    for i in range(num_images):
    # for i in [0]:
        topic, msg, t  = messages.next()
        yield ImageMsg(msg)

    bag.close()

def get_filename(idx):
    return '{:06d}'.format(idx) + '.png'

def save_images(bag_files, rgb_dir, bgr_dir):
    idx = 0
    for bag in bag_files:
        image_gen = read_images(bag)
        for img in image_gen:
            # squeezeNet model was trained with RGB, so stick with that.
            save_np_image(img.rgb, os.path.join(rgb_dir, get_filename(idx)))
            save_np_image(img.bgr, os.path.join(bgr_dir, get_filename(idx)))
            idx += 1

def makedir(out_dir):
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

def generate_training_images():
    bags = ['/home/eljefec/data/traffic_light_bag_files/just_traffic_light.bag',
            '/home/eljefec/data/traffic_light_bag_files/loop_with_traffic_light.bag']

    rgb_dir = './out/traffic_light_bag_files/rgb'
    bgr_dir = './out/traffic_light_bag_files/bgr'
    makedir(rgb_dir)
    makedir(bgr_dir)

    save_images(bags, rgb_dir, bgr_dir)

def record_classes(image_dir):
    dirs = [f for f in listdir(image_dir) if isdir(join(image_dir, f))]
    print(dirs)
    with open('classes.txt', 'w') as write_file:
        for d in dirs:
            path = os.path.join(image_dir, d)
            files = [f for f in listdir(path) if isfile(join(path, f))]
            for filename in files:
                line = filename + ',' + d
                print(line, file = write_file)

def sort_classes(filename):
    lines = []
    with open(filename) as f:
        for line in f:
            lines.append(line)
    print(len(lines))
    lines.sort()
    with open('sorted_classes.txt', 'w') as f:
        for line in lines:
            f.write(line)

def get_class(cls_string):
    cls_dict = { '0-unknown' : TrafficLight.UNKNOWN,
                 '1-red' : TrafficLight.RED,
                 '2-green' : TrafficLight.GREEN }
    return cls_dict[cls_string]

class TrafficLightExample:
    def __init__(self, line):
        tokens = line.split(',', 1)
        self.filename = tokens[0]
        self.cls = tokens[1]
        self.state = get_class(self.cls)

    def __repr__(self):
        return self.filename + ',' + self.cls

def get_examples(filename):
    examples = []
    with open(filename) as f:
        for line in f:
            line = line.rstrip()
            examples.append(TrafficLightExample(line))
    return examples

def split_train_val(filename):
    examples = get_examples(filename)
    examples = np.random.permutation(examples)
    val_split = len(examples) / 5
    train_examples = sorted(examples[val_split:], key=lambda ex : ex.filename)
    val_examples = sorted(examples[:val_split], key=lambda ex : ex.filename)
    print('val', len(val_examples), 'train', len(train_examples))
    return (train_examples, val_examples)

def setup_imageset(examples, setname, image_dir):
    setdir = os.path.join(image_dir, setname)
    makedir(setdir)
    classes = set()
    for ex in examples:
        classes.add(ex.cls)
    for cls in classes:
        makedir(os.path.join(setdir, cls))
    for ex in examples:
        clsdir = os.path.join(setdir, ex.cls)
        os.rename(os.path.join(image_dir, ex.filename), os.path.join(clsdir, ex.filename))

def resize_images(image_dir):
    files = [f for f in listdir(image_dir) if isfile(join(image_dir, f))]
    for filename in files:
        fullname = os.path.join(image_dir, filename)
        img = cv2.imread(fullname)
        img = cv2.resize(img, (IMAGE_WIDTH, IMAGE_HEIGHT))
        cv2.imwrite(fullname, img)

def setup_imagedata(train, val, image_dir):
    resize_images(image_dir)
    setup_imageset(train, 'train', image_dir)
    setup_imageset(val, 'val', image_dir)

if __name__ == '__main__':
    # generate_training_images()
    # record_classes('/home/eljefec/data/traffic_light_sim')
    # sort_classes('classes.txt')
    # get_examples('sorted_classes.txt')
    (train, val) = split_train_val('sorted_classes.txt')
    setup_imagedata(train, val, '/home/eljefec/data/traffic_light_sim/rgb')
