from tl_classifier import TLClassifier
from matplotlib import pyplot as plt

classifier = TLClassifier()
import cv2
image = cv2.imread('data/test_images_sim/left0988.jpg')

#plt.figure(figsize=(12, 8))
#plt.imshow(image)
#plt.show()

result = classifier.get_classification(image)
print('result: ', result)


