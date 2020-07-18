import csv
import cv2
import numpy as np
import sklearn
from math import ceil

# Fill the 'Data' list of applicable (image,measurement) tuple
# Data = [(image1, measurement1),
#         (image2, measurement2),
#         ... ...
#        ]

Data = []
TRAINING_DATA_DIR = './sim_img/'

num_of_images = 0
with open(TRAINING_DATA_DIR+'sim_images.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        image_fn = TRAINING_DATA_DIR+line[0]
        light_state = line[1]
        next_light_wp = line[2]
        
        Data.append((image_fn,light_state,next_light_wp))
        
        num_of_images += 1

    print('Total %d images' % num_of_images)

    
from sklearn.model_selection import train_test_split

# split samples into training and validation
# train_test_split performs shuffling by default
train_samples, validation_samples = train_test_split(Data, test_size=0.2)

print('%d training samples' % len(train_samples))
print('%d validation samples' % len(validation_samples))

from keras.utils import to_categorical

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
                
                image = cv2.imread(image_path)

                #image,angle = Augment_Data(image_path, angle)
                images.append(image)
                states.append(light_state)
                
            #print(states)
            one_hot_states = to_categorical(states)
            #print(one_hot_states)
            
            X_train = np.array(images)
            y_train = np.array(one_hot_states)
            yield sklearn.utils.shuffle(X_train,y_train)

'''
def Augment_Data(image_path,angle):
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # I heard that drive.py reads RGB
    
    # randomly flip the image
    if np.random.rand() > 0.5:
        image = np.fliplr(image)
        angle = -angle

    # randomly shift the image horizontally
    if 0: # for trial
        shift_range = 50
        shift = np.random.randint(shift_range) -(shift_range/2)
    
        m = np.float32([[1, 0, shift], [0, 1, 0]])
        rows, cols = image.shape[:2]
        image = cv2.warpAffine(image, m, (cols, rows))
    
        angle = angle + shift * (-0.02)
    
    return image,angle
'''    
            
batch_size = 32            
            
train_generator = generator(train_samples, batch_size = batch_size)
validation_generator = generator(validation_samples, batch_size = batch_size)
   
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Dropout, Activation, Cropping2D
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D

model = Sequential()
model.add(Lambda(lambda x: x /127.5 -1.0, input_shape=(600,800,3)))
#model.add(Cropping2D(cropping=((75,25),(0,0))))
model.add(Conv2D(24,(5,5))) #todo whats the parameter here
model.add(MaxPooling2D((2,2)))
#model.add(Dropout(0.8))
model.add(Activation('relu'))
model.add(Conv2D(36,(5,5))) #todo whats the parameter here
model.add(MaxPooling2D((2,2)))
model.add(Activation('relu'))
#model.add(Dropout(0.8))
model.add(Conv2D(48,(5,5))) #todo whats the parameter here
model.add(MaxPooling2D((2,2)))
model.add(Activation('relu'))
model.add(Conv2D(64,(3,3))) #todo whats the parameter here
model.add(MaxPooling2D((2,2)))
model.add(Activation('relu'))
#model.add(Dropout(0.8))
model.add(Flatten())
model.add(Dense(100))
#model.add(Dropout(0.2))
model.add(Dense(50))
model.add(Dense(4))

model.compile(loss='mse', optimizer='adam')

model.fit_generator(train_generator,\
                    steps_per_epoch= ceil(len(train_samples)/batch_size),\
                    validation_data=validation_generator,\
                    validation_steps=ceil(len(validation_samples)/batch_size),\
                    epochs=3, verbose=1)

model.save('model.h5')

