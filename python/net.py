from keras.models import Sequential, Model
from keras.layers import Flatten, Activation, Dense, Lambda, Dropout
from keras.layers.convolutional import Conv2D, SeparableConv2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.normalization import BatchNormalization
from keras.layers import Cropping2D
import numpy as np
import matplotlib.pyplot as plt
import aug
from utils import files_only
from keras.optimizers import Adam, SGD, Adadelta, Adagrad, Adamax, RMSprop, Nadam
from keras.callbacks import TensorBoard, EarlyStopping, ModelCheckpoint

def one_hot_int(y, num):
    if y is None or len(y)==0:
        return []

    np_y = np.array(y)
    h = np.zeros((len(y), num))
    h[np.arange(len(y)), np_y] = 1

    return h

class SingleLabel:
    def __init__(self, label_str, label_pos):
        self.label_str = label_str
        self.label_pos = label_pos

    def to_string(self):
        return self.label_str + ": " + str(self.label_pos)

class Labels:
    def __init__(self, labels_str):
        self.labels=[]

        for idx, label_str in enumerate(labels_str):
            self.labels.append(SingleLabel(label_str, idx))

def LightNetModel(shape, n_classes, normalize, big, keep_rate, drop, cropping = None):
    if (big):
        out_cv1 = 16
        out_cv2 = 32
        out_cv3 = 32
        out_cv4 = 32
        out_fc1 = 620
        out_fc2 = 184
    else:
        out_cv1 = 6
        out_cv2 = 6  # 20
        out_fc1 = 120
        out_fc2 = 84

    model = Sequential()
    init_shape = None
    if (cropping):
        model.add(Cropping2D(cropping=cropping, input_shape=shape))
    else:
        init_shape = shape

    if (normalize):
        model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=init_shape, name="Normalize"))
        model.add(Conv2D(out_cv1, (5, 5), activation='relu', name="conv1"))
    else:
        model.add(Conv2D(out_cv1, (5, 5), activation='relu', input_shape=init_shape, name="conv1"))

    model.add(MaxPooling2D(name="max1"))
    model.add(BatchNormalization(name="batch1"))

    if drop:
        model.add(Dropout(keep_rate, name="drop1"))
    model.add(Conv2D(out_cv2, (5, 5), activation='relu', name="conv2"))
    model.add(MaxPooling2D(name="max2"))
    model.add(BatchNormalization(name="batch2"))
    if drop:
        model.add(Dropout(keep_rate, name="drop2"))

    if big:
        model.add(Conv2D(out_cv3, (5, 5), activation='relu', name="conv3"))
    model.add(Flatten())
    model.add(Dense(out_fc1, name="fc1", activation='relu'))
    if drop:
        model.add(Dropout(keep_rate, name="drop4"))
    model.add(Dense(out_fc2, name="fc2", activation='relu'))
    if big:
        model.add(Dense(32, name="fc3", activation='relu'))

    model.add(Dense(n_classes, name="fc4", activation='softmax'))

    return model


def show_history(history_object):
    ### print the keys contained in the history object
    print(history_object.history.keys())

    ### plot the training and validation loss for each epoch
    plt.plot(history_object.history['loss'])
    plt.plot(history_object.history['val_loss'])
    plt.plot(history_object.history['acc'])
    plt.plot(history_object.history['val_acc'])
    plt.title('model mean squared error loss')
    plt.ylabel('mean squared error loss')
    plt.xlabel('epoch')
    plt.legend(['T loss', 'V loss', 'T acc', 'V acc'], loc='upper left')
    plt.show()

class LightNet:
    def __init__(self, base_dir, fast_training):
        self.base_dir = base_dir
        self.fast_training = fast_training
        self.num_classes = 7
        self.augment_validation = False
        self.validation_percentage = 0.15
        self.num_epochs = 100
        self.batch_size = 16
        self.optimizer = Adam(lr=0.001)  # 0.9130 - 0.9199 with Big
        self.data_labes = ["Off", "Green", "GreenLeft", "Red", "RedLeft", "Yellow", "YellowLeft", ]

    def get_class(self, dir, num_crop, label):
        label_hot = one_hot_int([label], self.num_classes)
        return aug.DataClass(files_only(self.base_dir + dir), num_crop, not self.fast_training, label_str=dir, label=label,
                             label_hot=label_hot[0], resize_to=(64, 64), cache=True)

    def create_dataset(self):
        data_classes = []

        data_classes.append(self.get_class(self.data_labes[0], 0 if self.fast_training else 5, 0))
        data_classes.append(self.get_class(self.data_labes[1], 0 if self.fast_training else 1, 1))
        data_classes.append(self.get_class(self.data_labes[2], 0 if self.fast_training else 5, 2))
        data_classes.append(self.get_class(self.data_labes[3], 0 if self.fast_training else 1, 3))
        data_classes.append(self.get_class(self.data_labes[4], 0 if self.fast_training else 1, 4))
        data_classes.append(self.get_class(self.data_labes[5], 0 if self.fast_training else 5, 5))
        data_classes.append(self.get_class(self.data_labes[6], 0 if self.fast_training else 5, 6))

        self.dataset=aug.DataSet(data_classes, self.validation_percentage, augment_validation=self.augment_validation)

    def show_distribution(self):
        for data_class in self.dataset.data_classes:
            (training, validation) = data_class.get_elems_augmented_split(validation_percentage=0.15,
                                                                          augment_validation=self.augment_validation)

            print(data_class.label_str + "[" + str(data_class.label) + " - " + str(
                data_class.label_hot) + "]: Training: " + str(len(training)) + " - Validation: " + str(len(validation)))

    def create_model(self, show_summary = False):
        print("model...")
        model = LightNetModel((64, 64, 1), self.num_classes, normalize=True, big=not self.fast_training, keep_rate=0.5, drop=True)

        loss = 'categorical_crossentropy'
        if show_summary:
            print(model.summary())
        print("Number of epochs: ", self.num_epochs)
        print("Compiling...")
        model.compile(loss=loss, optimizer=self.optimizer, metrics=['accuracy'])

        self.model = model

        return model

    def train(self, show_history):
        EARLY_STOPPING = EarlyStopping(monitor='val_loss', min_delta=0.0005, patience=3 if self.fast_training else 5, verbose=1,
                                       mode='auto')
        callbacks = [EARLY_STOPPING]
        checkpoint = ModelCheckpoint("model-ck.h5", monitor='val_loss', mode='min', verbose=1, save_best_only=True)
        callbacks.append(checkpoint)

        print("fitting")
        history_object = self.model.fit_generator(self.dataset.get_training_generator(batch_size=self.batch_size),
                                             steps_per_epoch=self.dataset.get_training_len() / self.batch_size,
                                             validation_data=self.dataset.get_validation_generator(batch_size=self.batch_size),
                                             validation_steps=self.dataset.get_validation_len() / self.batch_size,
                                             nb_epoch=self.num_epochs, callbacks=callbacks)
        print("saving")

        self.model.save('model.h5')

        with open("model.json", "w") as json_file:
            json_file.write(self.model.to_json())

        if (show_history):
            show_history(history_object)
