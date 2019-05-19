#!/usr/bin/env python

from light_classification.tl_classifier import TLClassifier
import os
import sys
import time
import cv2
from PIL import Image
import glob



if __name__ == '__main__':
    light_classifier = TLClassifier()
    while True:
        filename = raw_input("Filename:")
        if filename == 'q':
            sys.exit(0)

        for img in glob.glob('./test/*.jpg'):

            image = Image.open(img)  
            image = image.resize((32, 32))
            output = light_classifier.get_classification(image)

            print(output)
