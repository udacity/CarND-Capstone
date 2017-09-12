import numpy as np
import cv2
from keras.models import load_model
import os
from keras.models import Sequential
from keras.layers import Dropout, Flatten, Dense
from keras import applications
from keras.optimizers import SGD, Adam, RMSprop
import matplotlib.pyplot as plt
from keras.utils.np_utils import to_categorical
from keras.models import model_from_json

import matplotlib.pyplot as plt
os.chdir(r'C:\Users\priya\Documents\carnd\Term3\CarND-Capstone\TL_model')

img_path = 'preview/yellow_0.jpg'
from keras.preprocessing import image
img = image.load_img(img_path, grayscale = False, target_size=(256, 256))
img = image.img_to_array(img)

#Rescale image
img = img/255.
plt.imshow(img)
#Convert to a 4D tensor
image = np.expand_dims(img, axis=0)
print(image.shape)

# Run image through the same pipeline

num_classes = 4
# build the VGG16 network
model = applications.VGG16(include_top=False, weights='imagenet')

# get the bottleneck prediction from the pre-trained VGG16 model
bottleneck_prediction = model.predict(image)

# build top model
model = Sequential()
model.add(Flatten(input_shape=bottleneck_prediction.shape[1:]))
model.add(Dense(256, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(num_classes, activation='sigmoid'))

model.load_weights(top_model_weights_path)

# use the bottleneck prediction on the top model to get the final classification
class_predicted = model.predict_classes(bottleneck_prediction)

print(class_predicted)

#Decode prediction
inID = class_predicted[0]

class_dictionary = {}
class_dictionary[0] = 'green'
class_dictionary[1] = 'nolight'
class_dictionary[2] = 'red'
class_dictionary[3] = 'yellow'

label = class_dictionary[inID]

# get the prediction label
print("Image ID: {}, Label: {}".format(inID, label))

# display the predictions with the image
cv2.putText(img, "Predicted: {}".format(label), (10, 30), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 255), 2)

cv2.imshow("Classification", img)
cv2.waitKey(0)
cv2.destroyAllWindows()