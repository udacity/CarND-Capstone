from tl_classifier import TLClassifier
from scipy import misc
from os import path
from glob import glob
import random

def all_files(folder):
  files = list(glob(path.join(folder, "*.jpg")))
  return files

model = TLClassifier("models/tl_model")

green = [misc.imread(f) for f in all_files("../../data/camera_training/validation/green/")]
red = [misc.imread(f) for f in all_files(".../../data/camera_training/validation/red/")]
yellow = [misc.imread(f) for f in all_files("../../data/camera_training/validation/yellow/")] 
nolight = [misc.imread(f) for f in all_files("../../data/camera_training/validation/nolight/")]

images = [green, red, yellow, nolight]
labels = ["green", "red", "yellow", "nolight"]

for subset, label in zip(images, labels):
  for img in subset:
    assert model.get_classification(img) == label