import os
import keras
import rospy
import cv2
from PIL import Image
import cv2
import tensorflow as tf
import numpy as np
from keras.models import Sequential
from keras.layers import Dense, Conv2D, Flatten, MaxPool2D
from keras.preprocessing.image import ImageDataGenerator
import keras.backend as K
# from styx_msgs.msg import TrafficLight
from tensorflow.python.platform import app, flags
from yolo.yolo import YOLO
from yolo.yolo3.utils import letterbox_image

flags.DEFINE_bool('train_mode', False, 'run the script in training mode or not')
flags.DEFINE_integer('epochs', 50, 'number of ephocs for the training')
FLAGS = flags.FLAGS


# close all existing tensorflow session (if they exist)
if 'session' in locals() and session is not None:
    print('Close interactive session')
    session.close()

config = tf.ConfigProto()
config.gpu_options.allow_growth=True
config.gpu_options.per_process_gpu_memory_fraction = 0.8
sess = tf.Session(config=config)
K.set_session(sess)
GRAPH = tf.get_default_graph()

class TLClassifier(object):
    def __init__(self, load_checkpoint=True, is_site=False):
        self.img_counter = 0 # for yolo debug
        self.is_site = is_site
        if not is_site:
            rospy.loginfo("[tl_classifier] Loading simulator classification model") 
            self.model = self.simple_conv_net()
            checkpoint = '../../src/model_files/simulator_model_weights.08-0.03.hdf5'
            if load_checkpoint:
                self.model.load_weights(checkpoint)
            self.model._make_predict_function()
        
        else:
            self.yolo = YOLO()
            self.model = self.real_traffic_light_net()
            rospy.loginfo("[tl_classifier] Loading real life classification model") 
            # TODO - add trained real traffic light net that will work on carla
            checkpoint = None
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if not self.is_site:  
            image = image / 255.          
            image = np.expand_dims(image, 0)  # Add batch dimension.
            with GRAPH.as_default():
                pred = self.model.predict(image)
            pred = np.argmax(pred)
            if pred == 3:
                pred = 4
            rospy.loginfo("[tl_classifier] current traffic light classification: %d" % (pred)) 
            return pred

        else:
            with GRAPH.as_default():
                cropped_image = self.get_traffic_lights_crop_with_YOLO(image)
            if cropped_image is None:
                return 4
            # with GRAPH.as_default():
            #     pred = self.model.predict(image)
            # pred = np.argmax(pred)
            # if pred == 3:
            #     pred = 4
            # return pred
            return 4

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
    
    def real_traffic_light_net(self):
        # TODO - add the model that was trained on cropped images of real traffic lights
        pass

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

    def get_traffic_lights_crop_with_YOLO(self, img):
        '''
        gets rgb image
        returns the bounding box coordinates of a traffic light (left, top, right, bottom)S
        '''
        
        img = img[200:800,:, :]  # reduce the analysis area
        image = Image.fromarray(img)
        boxed_image = letterbox_image(image, tuple(reversed(self.yolo.model_image_size)))
        image_data = np.array(boxed_image, dtype='float32')
        image_data = image_data / 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        out_boxes, out_classes = self.yolo.sess.run(
            [self.yolo.boxes, self.yolo.classes],
            feed_dict={
                self.yolo.yolo_model.input: image_data,
                self.yolo.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })

        for i, c in reversed(list(enumerate(out_classes))):
            if c == 9:
                box = out_boxes[i]
                top, left, bottom, right = box
                top = max(0, np.floor(top + 0.5).astype('int32'))
                left = max(0, np.floor(left + 0.5).astype('int32'))
                bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
                right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
                cropped_image = img[top:bottom, left:right, :]
                rospy.loginfo("[tl_classifier] YOLO FOUND SOMETHING!!!!") 
                
                # for debug: save yolo cropped images
                # temp_folder = '/home/udacity-ros/data/test/'
                # img_name = 'img%05d.jpg' % self.img_counter
                # self.img_counter += 1
                # save_path = temp_folder + img_name
                # cv2.imwrite(save_path, cv2.cvtColor(cropped_image, cv2.COLOR_RGB2BGR))
                # return cropped_image
        
        rospy.loginfo("[tl_classifier] YOLO FOUND NOTHING!!!!") 
        return None

# def main(argv=None):
if FLAGS.train_mode:
    classifier = TLClassifier()
    classifier.train()

if __name__ == '__main__':
    app.run()

