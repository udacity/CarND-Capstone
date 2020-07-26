from keras.models import load_model
import h5py
import numpy as np
from keras import __version__ as keras_version
import tensorflow as tf
import cv2
import rospy
import glob

#classifier_mdl = "tl_classify_real_extracted.h5"
#classifier_mdl = "tl_classify_simulator_extracted.h5"
classifier_mdl = "tl_classify_mixed_extracted.h5"
# Load tl classifier
f = h5py.File(classifier_mdl, mode='r')
model_version = f.attrs.get('keras_version')
Keras_version = str(keras_version).encode('utf8')

if model_version != keras_version:
    rospy.loginfo('You are using Keras version ', keras_version,
                  ', but the model was built using ', model_version)

model = load_model(classifier_mdl)

Total = 0
Correct = 0

Data = []
files = glob.glob("real_extracted/Red/*")
for fname in files:
    Data.append((fname,0))
            
files = glob.glob("real_extracted/Yellow/*")
for fname in files:
    Data.append((fname,1))

files = glob.glob("real_extracted/Green/*")
for fname in files:
    Data.append((fname,2))
    

for i in range(len(Data)):
    fname = Data[i][0]
    truth = int(Data[i][1])
    
    img = cv2.imread(fname)
    dsize = (15, 30)

    tl_img = cv2.resize(img, dsize)

    image_array = np.asarray(tl_img)

  #  with self.graph.as_default():
    labels = model.predict(image_array[None,:,:,:])
    predict = np.argmax(labels)
                
    Total += 1
    if predict == truth:
        Correct += 1
    
print("correct predicted %d / %d" % (Correct, Total))    