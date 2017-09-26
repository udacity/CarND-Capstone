import tensorflow as tf
from os import path
import numpy as np
from scipy import misc
from styx_msgs.msg import TrafficLight


class RealModel(object):
  def __init__(self, model_checkpoint):
    self.sess = None
    self.checkpoint = model_checkpoint
    self.label2class = {
      TrafficLight.GREEN: 0,
      TrafficLight.YELLOW: 1,
      TrafficLight.RED: 2,
      TrafficLight.UNKNOWN: 3}
    self.class2label = {v:k for k, v in self.label2class.items()}
    tf.reset_default_graph()

  def predict(self, image):
    if self.sess == None:
      self.sess = tf.Session()
      saver = tf.train.import_meta_graph(self.checkpoint+".meta")
      saver.restore(self.sess, self.checkpoint)
      self.image, self.labels, self.training = tf.get_collection("model_input")
      self.logits, self.predictions = tf.get_collection("model_output")

    img = misc.imresize(image, (227, 227)).astype(np.float32)
    imgs = np.expand_dims(img, 0)

    prediction = self.sess.run(self.predictions,
                  feed_dict={
                    self.image: imgs,
                    self.training: False
    })

    prediction = int(np.squeeze(prediction))
    return self.class2label[prediction]