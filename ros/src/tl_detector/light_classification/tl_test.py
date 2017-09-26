import cv2
import glob
from tl_classifier import TLClassifier

light_classifier = TLClassifier('../light_classifier_model.h5')

for files in glob.glob('frame*.jpg'):
    image = cv2.imread(files)
    print(light_classifier.get_classification(image))
