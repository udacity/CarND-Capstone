import numpy as np
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Dropout, Flatten, Dense
from keras import applications
from keras.optimizers import SGD, Adam, RMSprop
import matplotlib.pyplot as plt
from keras.utils.np_utils import to_categorical
from keras.models import model_from_json

import cv2
import os
os.chdir(r'C:\Users\priya\Documents\carnd\Term3\CarND-Capstone\TL_model')
os.listdir()
loc = 'camera_training/train/red/left0140.jpg'
img = cv2.imread(loc)
print(img.shape) #1096x1368


# dimensions of our images.
img_width, img_height = 256, 256

top_model_weights_path = 'bottleneck_fc_model.h5'
train_data_dir = 'camera_training/train'
validation_data_dir = 'camera_training/validation'
nb_train_samples = 2944
nb_validation_samples = 1264
epochs = 30
batch_size = 16


def save_bottlebeck_features():
    datagen = ImageDataGenerator(rescale=1. / 255)

    # build the VGG16 network
    model = applications.VGG16(include_top=False, weights='imagenet')

    generator = datagen.flow_from_directory(
        train_data_dir,
        target_size=(img_width, img_height),
        batch_size=batch_size,
        class_mode=None,
        shuffle=False)
    bottleneck_features_train = model.predict_generator(
        generator, nb_train_samples // batch_size)
    np.save(open('bottleneck_features_train.npy', 'wb'),
            bottleneck_features_train)

    generator = datagen.flow_from_directory(
        validation_data_dir,
        target_size=(img_width, img_height),
        batch_size=batch_size,
        class_mode=None,
        shuffle=False)
    bottleneck_features_validation = model.predict_generator(
        generator, nb_validation_samples // batch_size)
    np.save(open('bottleneck_features_validation.npy', 'wb'),
            bottleneck_features_validation)


def train_top_model():
    train_data = np.load('bottleneck_features_train.npy')
    train_labels_temp = np.array(
        [0] * (1346) + [1] * (260) + [2]*(788) + [3]*(550))

    train_labels = to_categorical(train_labels_temp)


    validation_data = np.load('bottleneck_features_validation.npy')
    validation_labels_temp = np.array(
        [0] * (604) + [1] * (112) + [2] * (328) + [3] * (220))

    validation_labels = to_categorical(validation_labels_temp)

    model = Sequential()
    model.add(Flatten(input_shape=train_data.shape[1:]))
    model.add(Dense(256, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(4, activation='sigmoid'))

    sgd = SGD(lr=0.001, decay=1e-6, momentum=0.8, nesterov=True)
    adam = Adam(lr=0.001, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.0)
    rmsprop = RMSprop(lr=0.0005, rho=0.9, epsilon=1e-08, decay=0.0)

    model.compile(loss='categorical_crossentropy',
                  optimizer=rmsprop,
                  metrics=['accuracy'])

    history_object = model.fit(train_data, train_labels,
              epochs=epochs,
              batch_size=batch_size,
              validation_data=(validation_data, validation_labels))

    # serialize model to JSON
    model_json = model.to_json()
    with open("model.json", "w") as json_file:
        json_file.write(model_json)
    # serialize weights to HDF5
    model.save_weights(top_model_weights_path)
    print("Saved model to disk")
    return history_object


save_bottlebeck_features()
history_object = train_top_model()

## Plot performance

print(history_object.history.keys())
plt.plot(history_object.history['acc'])
plt.plot(history_object.history['val_acc'])
plt.title('model accuracy')
plt.ylabel('acc')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()

# 2944/2944 [==============================] - 4s - loss: 0.0795 - acc: 0.9715 - val_loss: 0.0141 - val_acc: 0.9984
