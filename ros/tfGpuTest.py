#!/bin/python

###
### Adapted from: https://www.tensorflow.org/programmers_guide/using_gpu
###   (accessed on 03/20/2018)
###

import tensorflow as tf

# Create a (minimal) graph
c = tf.constant([1.0])

# Creates a session with log_device_placement set to True.
sess = tf.Session(config=tf.ConfigProto(log_device_placement=True))

# Run graph
print(sess.run(c))
