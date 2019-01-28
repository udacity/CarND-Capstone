import keras
import rospy
import cv2
import numpy as np
from keras.models import Sequential
from keras.layers import Dense, Conv2D, Flatten, MaxPool2D
from keras.preprocessing.image import ImageDataGenerator
import tensorflow as tf
# from styx_msgs.msg import TrafficLight
from tensorflow.python.platform import app, flags
flags.DEFINE_bool('train_mode', False, 'run the script in training mode or not')
flags.DEFINE_integer('epochs', 50, 'number of ephocs for the training')
FLAGS = flags.FLAGS
GRAPH = tf.get_default_graph()

class TLClassifier(object):
    def __init__(self):
        self.model = self.simple_conv_net()
        self.load_checkpoint = '/home/udacity-ros/CarND-Capstone/ros/src/model_files/simulator_model_weights.08-0.03.hdf5'
        if self.load_checkpoint is not None:
            self.model.load_weights(self.load_checkpoint)
        
        self.model._make_predict_function()
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) / 255.
        image = np.reshape(image, (1,)+image.shape)

        with GRAPH.as_default():
            pred = self.model.predict(image)
        pred = np.argmax(pred)
        if pred == 3:
            pred = 4
        rospy.loginfo("[tl_classifier] current traffic light classification: %d" % (pred)) 
        return pred


        # return TrafficLight.UNKNOWN

    def simple_conv_net(self):
        model = Sequential()
        model.add(Conv2D(filters=32, kernel_size=5, strides=(2,2), input_shape=(600, 800, 3), activation='relu'))
        model.add(Conv2D(filters=32, kernel_size=3, activation='relu', strides=(2,2)))
        model.add(MaxPool2D(pool_size=(2,2)))
        model.add(Conv2D(filters=32, kernel_size=3, activation='relu', strides=(2,2)))
        model.add(MaxPool2D(pool_size=(2,2)))
        model.add(Conv2D(filters=64, kernel_size=3, activation='relu'))
        model.add(Conv2D(filters=64, kernel_size=3, activation='relu'))
        model.add(MaxPool2D(pool_size=(2,2)))
        model.add(Conv2D(filters=64, kernel_size=3, activation='relu'))
        model.add(Conv2D(filters=64, kernel_size=3, activation='relu'))
        model.add(Flatten())
        model.add(Dense(64, activation='relu'))
        model.add(Dense(4, activation='softmax'))
        model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

        return model
    
    def train(self, epoch_num=FLAGS.epochs):
        
        train_data_folder = '/home/udacity-ros/data/udacity-simulator-data/simulator-images/'
        val_data_folder = '/home/udacity-ros/data/udacity-simulator-data/simulator-images-val/'
        
        train_datagen = ImageDataGenerator(rescale=1./255, shear_range=0.2, zoom_range=0.2, horizontal_flip=True)
        train_generator = train_datagen.flow_from_directory(train_data_folder, 
                                                            target_size=(600, 800),
                                                            batch_size=8, class_mode='categorical')
        val_datagen = ImageDataGenerator(rescale=1./255)
        val_generator = val_datagen.flow_from_directory(val_data_folder, target_size=(600, 800),
                                                        batch_size=8, class_mode='categorical')

        # Path to save weights
        filepath = './model_files/simulator_model_weights.{epoch:02d}-{val_loss:.2f}.hdf5'
        model_checkpoint = keras.callbacks.ModelCheckpoint(filepath, monitor='val_loss', verbose=0,
                                                           save_best_only=True, save_weights_only=False, 
                                                           mode='auto', period=1)
        lr_reduce = keras.callbacks.ReduceLROnPlateau(monitor='val_loss', factor=0.33, patience=3,
                                                     verbose=0, mode='auto', cooldown=0, min_lr=0)
        self.model.fit_generator(train_generator, steps_per_epoch=720, validation_data=val_generator, validation_steps=175,
                                 epochs=epoch_num, callbacks=[model_checkpoint, lr_reduce])


# def main(argv=None):
if FLAGS.train_mode:
    classifier = TLClassifier()
    # ipdb.set_trace()
    classifier.train()

if __name__ == '__main__':
    app.run()

