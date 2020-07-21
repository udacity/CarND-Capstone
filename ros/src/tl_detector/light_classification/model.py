import csv
import cv2
import numpy as np
import sklearn
from math import ceil
from keras.utils import to_categorical
import keras

Data = []
TRAINING_DATA_DIR = './sim_img/'

number_of_all_class = [0,0,0,0]

with open(TRAINING_DATA_DIR+'sim_images.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        image_fn = TRAINING_DATA_DIR+line[0]
        light_state = int(line[1])
        next_light_wp = int(line[2])
        
        if light_state == 0 and next_light_wp < 200:
            repeat = 5
            is_red_light = 1
        elif light_state == 1 and next_light_wp < 200:
            repeat = 3
            is_red_light = 0
        elif light_state == 2 and next_light_wp < 200:
            repeat = 2
            is_red_light = 0
        elif light_state == 3 and next_light_wp > 600:
            repeat = 1
            is_red_light = 0
        else:
            repeat = 0
            
            
        for i in range(repeat):
            number_of_all_class[int(light_state)] +=1
            Data.append((image_fn,light_state,next_light_wp))
            
            
            #number_of_all_class[is_red_light] +=1
            #Data.append((image_fn,is_red_light,next_light_wp))
        
    print('Total %d images' % len(Data))
    print(number_of_all_class)

    
from sklearn.model_selection import train_test_split

# split samples into training and validation
# train_test_split performs shuffling by default
train_samples, validation_samples = train_test_split(Data, test_size=0.2)

print('%d training samples' % len(train_samples))
print('%d validation samples' % len(validation_samples))


def generator(samples, aug_data = False, batch_size=32, first_batch = True):
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
                
                dsize = (200, 150)

                if aug_data is False:
                    image = cv2.imread(image_path)
                    
                else:
                    image = Augment_Data(image_path)
                    
                image = cv2.resize(image, dsize)
                #image = image.astype(np.float16)
                
                images.append(image)
                states.append(light_state)
                
            one_hot_states = to_categorical(states,num_classes=4)
            if first_batch:
                print(one_hot_states)
                first_batch = False
            X_train = np.array(images)
            y_train = np.array(one_hot_states)
            yield sklearn.utils.shuffle(X_train,y_train)


def Augment_Data(image_path):
    image = cv2.imread(image_path)
    
    # randomly flip the image
    if 1:
        if np.random.rand() > 0.5:
            image = np.fliplr(image)

    # randomly shift the image horizontally and veritcally
    if 1:
        shift_x = np.random.randint(image.shape[1])
        image = np.roll(image, shift_x, axis=1)
        
        shift_y = np.random.randint(image.shape[0])
        image = np.roll(image, shift_y, axis=0)

    result = image#astype(np.float16)

    return result
    
            
batch_size = 64            
            
train_generator = generator(train_samples, aug_data = True, batch_size = batch_size)
validation_generator = generator(validation_samples, aug_data = False, batch_size = batch_size)
   
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Dropout, Activation, Cropping2D
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D

model = Sequential()
model.add(Lambda(lambda x: x /127.5 -1.0, input_shape=(150,200,3)))
#model.add(Cropping2D(cropping=((75,25),(0,0))))
model.add(Conv2D(16,(5,5))) #todo whats the parameter here
model.add(MaxPooling2D((2,2)))
#model.add(Dropout(0.5))
model.add(Activation('relu'))
model.add(Conv2D(32,(5,5))) #todo whats the parameter here
model.add(MaxPooling2D((2,2)))
model.add(Activation('relu'))
model.add(Dropout(0.2))
model.add(Conv2D(64,(3,3))) #todo whats the parameter here
model.add(MaxPooling2D((2,2)))
model.add(Activation('relu'))
#model.add(Conv2D(256,(3,3))) #todo whats the parameter here
#model.add(MaxPooling2D((2,2)))
#model.add(Activation('relu'))
#model.add(Dropout(0.8))
model.add(Flatten())
model.add(Dense(50))
model.add(Dropout(0.2))
#model.add(Dense(50))
model.add(Dense(4,activation='softmax'))

#,activation='softmax'
#categorical_crossentropy
optimizer = keras.optimizers.Adam(lr=0.0001)
model.compile(loss='categorical_crossentropy', optimizer=optimizer, metrics=['accuracy'])

model.fit_generator(train_generator,\
                    steps_per_epoch= ceil(len(train_samples)/batch_size),\
                    validation_data=validation_generator,\
                    validation_steps=ceil(len(validation_samples)/batch_size),\
                    epochs=50, verbose=1)

model.save('sim_model.h5')
