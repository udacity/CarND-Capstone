import tensorflow as tf

from tensorflow import saved_model

import glob

from scipy import misc
import numpy as np

import re

class inference(object):
    def __init__(self, sess, model_path):
        saved_model.loader.load(sess, [saved_model.tag_constants.SERVING], model_path)
        graph = tf.get_default_graph()

        self.input_layer = graph.get_tensor_by_name('input:0')
        self.output = graph.get_tensor_by_name('output:0')
    
    def pred(self, sess, imgs):
        output = sess.run([self.output], feed_dict={self.input_layer: imgs})
        return np.argmax(output[0], axis=1)

def main():
    with tf.Session() as sess:
        model = inference(sess, "../ros/src/tl_detector/saved_model)

        data_path = "test image path"

        images = glob.glob(data_path)
        X = []
        y = []

        for img_path in images: 
            img =  misc.imread(img_path)/255.0
            X.append(img)
            label = re.findall('(\d+_(\d+))', img_path)[0][1]
            label = int(label)
            if label == 4:
                label = 3
            y.append(label)
        n = len(X)
        print('read data success!')
        print('data size: ' + str(n))

        result = model.pred(sess, np.array(X))

        print(result)
        print()
        print(y)

if __name__ == '__main__':
    main()
