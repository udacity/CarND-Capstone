from keras.models import Model
from keras.applications import mobilenet
import csv
import numpy as np
import h5py
import scipy
from keras.models import load_model
import time

path_labels = {}

with open('file_labels.csv', 'r') as csvfile:
  reader = csv.reader(csvfile)
  for row in reader:
    path_labels[row[0]] = row[1]

image_size = (224, 224, 3)

image_paths = list(path_labels.keys())
label_categories = {'None': 0, 'Red': 1, 'Yellow': 2, 'Green': 3}

def load_image(image_path):
  return scipy.misc.imresize(scipy.misc.imread(image_path), image_size)

def get_category(image_path):
  color = path_labels[image_path]
  category = label_categories[color]

  return category

def get_one_hot(image_path):
  category = get_category(image_path)
  y_one_hot = [int(index == category) for index in range(num_classes)]

  return y_one_hot
  
model = load_model('model.h5', custom_objects={
                   'relu6': mobilenet.relu6,
                   'DepthwiseConv2D': mobilenet.DepthwiseConv2D})

#for i, layer in enumerate(model.layers):
#   print(i, layer.name)

# predict
matches = []
test_count = 1000
images = []
categories = []
#image_paths = ['rosbag_images/frame0001.jpg', 'rosbag_images/frame0050.jpg', 'rosbag_images/frame0700.jpg', 'rosbag_images/frame0900.jpg']
#categories = [1, 1, 0, 0]
for path in np.random.choice(image_paths, test_count):
  image = load_image(path)
  category = get_category(path)
  images.append(image)
  categories.append(category)

start = time.time()
for i in range(len(images)):
  prediction = np.argmax(model.predict(np.array([images[i]]))[0])
  category = categories[i]
  #print(path, category, prediction)
  matches.append(prediction == category)
end = time.time()

print("seconds elapsed: ", end - start)

number_true = matches.count(True)
print("Accuracy: ", float(number_true) / test_count)
