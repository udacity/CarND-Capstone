from keras.models import Sequential
from keras.layers import Dense, Conv2D, Flatten, MaxPool2D
from keras.preprocessing.image import ImageDataGenerator
# from styx_msgs.msg import TrafficLight

from tensorflow.python.platform import app, flags
flags.DEFINE_bool('train_mode', False, 'run the script in training mode or not')
FLAGS = flags.FLAGS

class TLClassifier(object):
    def __init__(self, train_mode=False):
        #TODO load classifier
        self.train_mode = train_mode
        self.load_checkpoint = None
        self.train_data_folder = '/home/udacity-ros/data/udacity-simulator-data/simulator-images/'
        self.model = self.simple_conv_net()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
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
    
    def train(self, epoch_num=50):
        modelCheckpoint = keras.callbacks.ModelCheckpoint(filepath, monitor='val_loss', verbose=0, save_best_only=True,
                                                          save_weights_only=False, mode='auto', period=1)
        train_datagen = ImageDataGenerator(rescale=1./255, shear_range=0.2, zoom_range=0.2, horizontal_flip=True)
        train_generator = train_datagen.flow_from_directory(self.train_data_folder, target_size=(600, 800), batch_size=8, class_mode='categorical')


        self.model.fit_generator(train_generator, steps_per_epoch=600, epochs=epoch_num)


def main(argv=None):
    if FLAGS.train_mode:
        classifier = TLClassifier(train_mode=True)
        classifier.train()
if __name__ == '__main__':
    app.run()

