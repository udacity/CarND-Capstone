# Source: https://github.com/mynameisguy/TrafficLightChallenge-DeepLearning-Nexar

from keras.preprocessing import image
from consts import DATASET_FOLDER, BATCH_SIZE, IMAGE_HEIGHT, IMAGE_WIDTH


class dataHandler:
    def getGenerators(self, batch_size=BATCH_SIZE):
        datagen = image.ImageDataGenerator(rescale=1.0/255, shear_range=0.2,
                                           zoom_range=0.2,
                                           rotation_range=5,
                                           horizontal_flip=True)

        val_datagen = image.ImageDataGenerator(rescale=1.0/255)


        print("creating train generator")
        train_generator = datagen.flow_from_directory(
            DATASET_FOLDER + '/train',
            target_size=(IMAGE_HEIGHT, IMAGE_WIDTH),
            batch_size=batch_size,
            class_mode='categorical')

        print("creating validation generator")
        validation_generator = val_datagen.flow_from_directory(
            DATASET_FOLDER + '/test',
            target_size=(IMAGE_HEIGHT, IMAGE_WIDTH),
            batch_size=batch_size,
            class_mode='categorical')

        return train_generator, validation_generator
