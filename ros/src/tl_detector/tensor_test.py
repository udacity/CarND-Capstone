#!/usr/bin/env python

from light_classification.tl_classifier import TLClassifier
import os
import sys
import time
import cv2



if __name__ == '__main__':
    light_classifier = TLClassifier()
    while True:
        filename = input("Filename:")
        if filename == 'q':
            sys.exit(0)
        start = time.time()
        img_filename = './00000013.png'
        test_image = cv2.imread('./00000013.png')
        output_img = light_classifier.detect_traffic_lights(test_image)
        
        end = time.time()
        print("Time: %s s"%(end-start))
