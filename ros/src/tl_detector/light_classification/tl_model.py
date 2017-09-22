import os
import json
import pickle
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers import Convolution2D, MaxPooling2D, AveragePooling2D


X_train = IMAGE_DATA['train_dataset']
y_train = IMAGE_DATA['train_labels']
X_val = IMAGE_DATA['val_dataset']
y_val = IMAGE_DATA['val_labels']

batch_size = 100
nb_classes = 1
nb_epoch = 35

X_train = X_train.astype('float32')
X_test = X_val.astype('float32')
print(X_train.shape[0], 'train samples')
print(X_val.shape[0], 'test samples')


#---Model-Definition:

input_shape = X_train.shape[1:]

model = Sequential()

#Start with 4 Convolutiional Layers to recognize the image
model.add(Convolution2D(60, 16, 16, subsample=(2, 2), border_mode='same', input_shape=input_shape, activation='relu', dim_ordering='tf'))
model.add(Convolution2D(100, 2, 2, border_mode='same', input_shape=input_shape, activation='relu', dim_ordering='tf'))
model.add(MaxPooling2D(pool_size=(2, 2), border_mode='same', dim_ordering='tf'))
model.add(Convolution2D(140, 2, 2, border_mode='same', input_shape=input_shape, activation='relu', dim_ordering='tf'))
model.add(MaxPooling2D(pool_size=(2, 2), border_mode='same', dim_ordering='tf'))
model.add(Convolution2D(180, 6, 6, border_mode='same', input_shape=input_shape, activation='relu', dim_ordering='tf'))
model.add(MaxPooling2D(pool_size=(2, 2), border_mode='same', dim_ordering='tf'))
model.add(Dropout(0.25))

#Flatten the Matrix to a Vektor and run 3 RELU Layers
model.add(Flatten())
model.add(Dense(180, name="hidden1"))
model.add(Activation('relu'))
model.add(Dense(60, name="hidden2"))
model.add(Activation('relu'))
model.add(Dense(20, name="hidden3"))
model.add(Activation('relu'))
model.add(Dropout(0.5))
model.add(Dense(2, name="Traffic_Light_State"))
model.add(Activation('softmax'))

model.summary()

model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])
history = model.fit(X_train, y_train, batch_size=batch_size, nb_epoch=nb_epoch, verbose=1, validation_data=(X_val, y_val))

json_string = model.to_json()
with open('./tl_model.json', 'w') as outfile:
    json.dump(json_string, outfile)

model.save_weights('./tl_model.h5')