from keras.applications.mobilenet import MobileNet
from keras.preprocessing import image as preprocess
from keras.models import Model
from keras.layers import Dense, GlobalAveragePooling2D, Flatten, Dropout
from keras import backend as K
from keras.utils import to_categorical
import csv
import random
import numpy as np
import h5py
import scipy
#print np.__version__ 
#print np.__path__

path_labels = {}

with open('colors_only_labels.csv', 'r') as csvfile:
  reader = csv.reader(csvfile)
  for row in reader:
    if row[1] != 'None':
      path_labels[row[0]] = row[1]

image_size = (224, 224, 3)
#image_size = (200, 150, 3)
batch_size = 16
num_classes = 4
epochs = 96


base_model = MobileNet(
  alpha=0.25,          # adjust down to make model smaller/faster by reducing filter count
  depth_multiplier=1,  # adjust down to make model smaller/faster by reducing resolution per layer
  weights='imagenet',
  #weights=None,
  include_top=False,
  #classes=num_classes,
  input_shape=image_size
)

x = base_model.output
x = Flatten()(x)

# intermediate layers
x = Dense(64, activation='relu')(x)
x = Dropout(0.1)(x)
x = Dense(32, activation='relu')(x)
x = Dropout(0.1)(x)
# and a prediction layer
predictions = Dense(num_classes, activation='softmax')(x)
#predictions = base_model.output

model = Model(inputs=base_model.input, outputs=predictions)

# freeze all base model layers and their weights
# not doing this for now, as we'll train the full network
#for layer in base_model.layers:
#    layer.trainable = False
#for layer in model.layers[:82]:
#   layer.trainable = False
#for layer in model.layers[82:]:
#   layer.trainable = True

# compile the model (*after* setting layers to non-trainable)
model.compile(optimizer='adam', loss='categorical_crossentropy')

image_paths = list(path_labels.keys())
label_categories = {'Red': 0, 'Yellow': 1, 'Green': 2}

def load_image(image_path):
  return scipy.misc.imresize(scipy.misc.imread(image_path), image_size)

def get_one_hot(image_path):
  color = path_labels[image_path]
  category = label_categories[color]
  y_one_hot = [int(index == category) for index in range(num_classes)]

  return y_one_hot
  

def get_image_batches(batch_size):

  while True:
    random.shuffle(image_paths)
    for batch_i in range(0, len(image_paths), batch_size):
      x = []
      y = []
      for image_path in image_paths[batch_i:batch_i+batch_size]:
        image = load_image(image_path)

        # augment data
        # rotate up to 2 degrees
        image = preprocess.random_rotation(image, 2, row_axis=0, col_axis=1, channel_axis=2)
        # randomly shift up to 20%
        image = preprocess.random_shift(image, 0.2, 0.2, row_axis=0, col_axis=1, channel_axis=2)
        # randomly zoom in up to 20%
        image = preprocess.random_zoom(image, (0.8, 0.8), row_axis=0, col_axis=1, channel_axis=2)
        #adjust brightness
        image = preprocess.random_brightness(image, (0.8, 1.2))
        # randomly flip horizontally
        if np.random.random() > 0.5:
          image = preprocess.flip_axis(image, 1)

        x.append(image)
        y.append(get_one_hot(image_path))

      yield np.array(x), np.array(y)


model.fit_generator(
  get_image_batches(batch_size),
  steps_per_epoch=len(image_paths)/batch_size,
  epochs=epochs
)

model.save('model.h5')
