import numpy as np
import sys
import os
import tensorflow as tf
from tensorflow.contrib import layers

import glob

from scipy import misc
import numpy as np

import re

from tqdm import tqdm

initializer = layers.xavier_initializer()

n_class = 4

from imgaug import augmenters as iaa

from datetime import datetime

def conv2d(input_layer, kernel_size, input_size, output_size):
    w_shape = [kernel_size, kernel_size, input_size, output_size]
    w = tf.Variable(initializer(w_shape))
    conv = tf.nn.conv2d(input_layer, w, [1, 1, 1, 1], 'SAME')
    return tf.nn.relu(conv)

def max_pool(input_layer):
    ksize = [1, 2, 2, 1]
    stride = [1, 2, 2, 1]
    return tf.nn.max_pool(input_layer, ksize, stride, 'SAME')

def dense(input_layer, input_size, output_size, activate):
    fc_w = tf.Variable(initializer([input_size, output_size]))
    fc_b = tf.Variable(initializer([output_size]))

    output = tf.add(tf.matmul(input_layer, fc_w), fc_b)
    if activate:
        output = tf.nn.relu(output)
    return output

def build_model(learning_rate):
    input_layer = tf.placeholder(tf.float32, [None, 600, 800, 3], name='input')
    
    label = tf.placeholder(tf.int32, [None])

    conv = conv2d(input_layer, 3, 3, 16)
    pool = conv
    for i in range(6):
        conv = conv2d(pool, 3, 16, 16)
        pool = max_pool(conv)
    pool_shape = pool.shape.as_list()
    flatten_size = pool_shape[1] * pool_shape[2] * pool_shape[3]
    flatten = tf.reshape(pool, [-1, flatten_size])

    n_fc = 128
    fc = dense(flatten, flatten_size, n_fc, True)
    output = dense(fc, n_fc, n_class, False)
    output = tf.identity(output, name='output')

    loss = tf.nn.softmax_cross_entropy_with_logits_v2(logits=output, labels=tf.one_hot(label, n_class))

    loss = tf.reduce_mean(loss)

    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(loss)

    return input_layer, label, loss, optimizer, output

def aug_img(imgs):
    seq = iaa.Sequential([
        iaa.Fliplr(0.5),
        iaa.Sometimes(0.5, iaa.CropAndPad(
            percent=(-0.1, 0.1),
        )),
        iaa.Sometimes(0.5, iaa.Affine(
            scale=(0.8, 1.2),
            rotate=(-15, 15)
        )),
    ])

    return seq.augment_images(imgs)
def read_data(batch_size):
    data_path = "../ros/src/tl_detector/simulator_images"

    images = glob.glob(data_path)

    n = len(images)
    print('read data success!')
    print('data size: ' + str(n))
    while True:
        order = np.random.permutation(n)

        for batch in range(0, n, batch_size):
            if batch + batch_size > n:
                break
            batch_x = []
            batch_y = []

            for i in range(batch, batch + batch_size):
                img_path = images[order[i]]
                img =  misc.imread(img_path)/255.0
                

                label = re.findall('(\d+_(\d+))', img_path)[0][1]
                label = int(label)
                if label == 4:
                    label = 3
                batch_x.append(img)
                batch_y.append(label)
            yield aug_img(batch_x), batch_y

def read_real_data(batch_size):
    data_path = "..." # Todo

    images = glob.glob(data_path)

    n = len(images)
    print('read data success!')
    print('data size: ' + str(n))
    while True:
        order = np.random.permutation(n)

        for batch in range(0, n, batch_size):
            if batch + batch_size > n:
                break
            batch_x = []
            batch_y = []

            for i in range(batch, batch + batch_size):
                img_path = images[order[i]]
                img =  misc.imread(img_path)/255.0
                img =  np.resize(img, (600, 800, 3))
                label = 2
                if(len(re.findall('unidentified', img_path)) > 0): label = 3
                if(len(re.findall('green', img_path))> 0): label = 2
                if(len(re.findall('nolight', img_path)) > 0): label = 3
                if(len(re.findall('red', img_path)) > 0): label = 0
                if(len(re.findall('yellow', img_path)) > 0): label = 1

                label = int(label)

                batch_x.append(img)
                batch_y.append(label)
            yield aug_img(batch_x), batch_y            

def main():
    n_epoc = 10
    batch_size = 32
    bath_per_epoc = 3000 // batch_size
    data_reader = read_data(batch_size)
    learning_rate = 0.002
    
    with tf.Session() as sess:
        
        input_layer, label, loss, optimizer, output = build_model(learning_rate)

        sess.run(tf.global_variables_initializer())
        for epoc in range(n_epoc):
            print('epoc ' + str(epoc))

            for _ in tqdm(range(bath_per_epoc), ncols=75):
                x, y = next(data_reader)

                feed_dict = {
                                input_layer : x,
                                label : y
                            }
                batch_loss, _ = sess.run([loss, optimizer], feed_dict = feed_dict)
            print(batch_loss)

        now = datetime.now()
        model_path = str.format('../ros/src/tl_detector/saved_model/{0}{1}{2}_{3}{4}{5}', now.year, now.month, now.day, now.hour, now.minute, now.second)
        tf.saved_model.simple_save(sess, model_path, inputs={'input': input_layer}, outputs={'output': output})

if __name__ == "__main__":
    main()
