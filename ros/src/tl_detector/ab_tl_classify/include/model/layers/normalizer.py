import keras.backend as K
from keras.layers.core import Layer

import numpy as np

class Normalizer(Layer):
    def __init__(self, range = (-1, 1.0), **kwargs):
        self.range = range
        super(Normalizer, self).__init__(**kwargs)
    
    def build(self, input_shape):
        super(Normalizer, self).build(input_shape)  # Be sure to call this somewhere!

    def call(self, x):
        return ((x - K.min(x)) / (255 - K.min(x))) * (self.range[1] - self.range[0]) + self.range[0]

    def compute_output_shape(self, input_shape):
        return input_shape
