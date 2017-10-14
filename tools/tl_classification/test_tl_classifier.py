from tl_classifier import TLClassifier
from scipy import misc
from os import path
from glob import glob
import random

def random_file(folder):
  files = list(glob(path.join(folder, "*.jpg")))
  choice = random.randint(0, len(files)-1)
  return files[choice]

model = TLClassifier("models/tl_model")

green = misc.imread(random_file("../../data/camera_training/validation/green/"))
red = misc.imread(random_file("../../data/camera_training/validation/red/"))
yellow = misc.imread(random_file("../../data/camera_training/validation/yellow/"))
nolight = misc.imread(random_file("../../data/camera_training/validation/nolight/"))

images = [green, red, yellow, nolight]
# images = [misc.imresize(img, [800, 600]) for img in images]
labels = ["green", "red", "yellow", "nolight"]

for img, label in zip(images, labels):
  assert model.get_classification(img) == label