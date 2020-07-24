import csv
import cv2
import numpy as np
import sklearn
from math import ceil
from keras.utils import to_categorical
import keras
import os
import glob

Data = []

number_of_all_class = [0,0,0]

#files = glob.glob("extract_img/SIM_RED/*.jpeg")
files = glob.glob("real_img/Red/*")
for fname in files:
    number_of_all_class[0] +=1
    Data.append((fname,0))
            
#files = glob.glob("extract_img/SIM_YELLOW/*.jpeg")
files = glob.glob("real_img/Yellow/*")
for fname in files:
    number_of_all_class[1] +=1
    Data.append((fname,1))

#files = glob.glob("extract_img/SIM_GREEN/*.jpeg")
files = glob.glob("real_img/Green/*")
for fname in files:
    number_of_all_class[2] +=1
    Data.append((fname,2))
    
print('Total %d images' % len(Data))
print(number_of_all_class)

    
from sklearn.model_selection import train_test_split

# split samples into training and validation
# train_test_split performs shuffling by default
train_samples, validation_samples = train_test_split(Data, test_size=0.2)

def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1:
        samples = sklearn.utils.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            
            images = []
            states = []
            
            for each_sample in batch_samples:
                image_path = each_sample[0]
                light_state = each_sample[1]
                
                dsize = (15, 30)

                image = cv2.imread(image_path)

                image = cv2.resize(image, dsize)
                
                images.append(image)
                
                states.append(light_state)
                
            one_hot_states = to_categorical(states,num_classes=3)

            X_train = np.array(images)
            y_train = np.array(one_hot_states)
            yield sklearn.utils.shuffle(X_train,y_train)

            
batch_size = 32            
            
train_generator = generator(train_samples,batch_size = batch_size)
validation_generator = generator(validation_samples, batch_size = batch_size)
   
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Dropout, Activation, Cropping2D
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D

model = Sequential()
model.add(Lambda(lambda x: x /127.5 -1.0, input_shape=(30,15,3)))
#model.add(Cropping2D(cropping=((75,25),(0,0))))
model.add(Conv2D(5,(2,2))) #todo whats the parameter here
model.add(MaxPooling2D((2,2)))
#model.add(Dropout(0.5))
model.add(Activation('relu'))
#model.add(Conv2D(32,(5,5))) #todo whats the parameter here
#model.add(MaxPooling2D((2,2)))
#model.add(Activation('relu'))
#model.add(Dropout(0.2))
#model.add(Conv2D(64,(3,3))) #todo whats the parameter here
#model.add(MaxPooling2D((2,2)))
#model.add(Activation('relu'))
#model.add(Conv2D(256,(3,3))) #todo whats the parameter here
#model.add(MaxPooling2D((2,2)))
#model.add(Activation('relu'))
#model.add(Dropout(0.8))
model.add(Flatten())
model.add(Dense(50))
model.add(Dropout(0.2))
#model.add(Dense(50))
model.add(Dense(3,activation='softmax'))

#,activation='softmax'
#categorical_crossentropy
optimizer = keras.optimizers.Adam(lr=0.001)
model.compile(loss='categorical_crossentropy', optimizer=optimizer, metrics=['accuracy'])

model.fit_generator(train_generator,\
                    steps_per_epoch= ceil(len(train_samples)/batch_size),\
                    validation_data=validation_generator,\
                    validation_steps=ceil(len(validation_samples)/batch_size),\
                    epochs=10, verbose=1)

model.save('tl_classify_real_extraced.h5')
