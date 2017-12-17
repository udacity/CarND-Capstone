import cv2
import random
import numpy as np
import sklearn
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from skimage.exposure import equalize_hist
from skimage.exposure import equalize_adapthist
from skimage.exposure import rescale_intensity
from skimage.restoration import denoise_tv_bregman
from skimage.restoration import denoise_tv_chambolle

def resize_to_1_if_required(img):
    shape = np.shape(img)

    if (len(shape)==3):
        return img

    return np.reshape(img, (shape[0], shape[1], 1))

def bgr_to_gray(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return resize_to_1_if_required(img)

def load_image_gray(path):
    img = cv2.imread(path)

    return bgr_to_gray(img)



# Augmenter returning the original file
class AugFromFile:
    def __init__(self, file, label=None, label_hot=None, label_str=None, cache = False, resize_to=None):
        self.file = file
        self.resize_to = resize_to
        self.cache = cache
        self.img = self.load_and_resize() if cache else None
        self.label = label
        self.label_hot = label_hot
        self.label_str = label_str

    def get(self):
        return self.img if self.cache else self.load_and_resize()

    def load_and_resize(self):
        bgr = False
        img = cv2.imread(self.file)

        if not bgr:
            img = bgr_to_gray(img)

        if self.resize_to:
            img = cv2.resize(img, self.resize_to, interpolation=cv2.INTER_CUBIC)

        # img = equalize_hist(img.squeeze())
        return resize_to_1_if_required(img)

    def to_string(self):
        return self.file + " Label: " + str(self.label_str) + " - " + str(self.label) + " - Hot: " + str(self.label_hot)

# Augmenter mirroring orizontally
class AugMirrorH:
    def __init__(self, parent_aug):
        self.parent_aug = parent_aug
        self.label = parent_aug.label
        self.label_hot = parent_aug.label_hot
        self.label_str = parent_aug.label_str

    def get(self):
        img = cv2.flip(self.parent_aug.get(), 1)
        img = resize_to_1_if_required(img)

        return img

    def to_string(self):
        return "Mirror of " + self.parent_aug.to_string()

# Augmenter cropping
class AugCrop:
    def __init__(self, parent_aug):
        self.parent_aug = parent_aug
        self.label = parent_aug.label
        self.label_hot = parent_aug.label_hot
        self.label_str = parent_aug.label_str

    def get(self):
        img = self.parent_aug.get()
        (rows, columns, channels) = np.shape(img)

        horiz_w = int(columns * (0.6 + 0.4*random.random()))
        horiz_x = int((columns-horiz_w) * random.random())

        vert_h = int(rows * (0.6 + 0.4 * random.random()))
        vert_y = int((rows - vert_h) * random.random())

        img = img[vert_y:vert_y+vert_h, horiz_x:horiz_x+horiz_w]
        img = cv2.resize(img, (rows, columns), interpolation=cv2.INTER_CUBIC)

        img = resize_to_1_if_required(img)

        return img

    def to_string(self):
        return "Crop of " + self.parent_aug.to_string()

# Class representing a single type of sample (e.g. green light)
class DataClass:
    def __init__(self, files, num_crops, add_mirror, label_str=None, label=None, label_hot=None, resize_to=None, cache=False):
        self.files = files
        self.num_crops = num_crops
        self.add_mirror = add_mirror
        self.total_aug = 1 + num_crops
        self.label=label
        self.label_hot = label_hot
        self.label_str = label_str
        self.resize_to=resize_to
        self.cache = cache

        if add_mirror:
            self.total_aug *= 2

    def get_orig_file(self, orig_file):
        return AugFromFile(orig_file, label_str=self.label_str, label=self.label, label_hot=self.label_hot, resize_to=self.resize_to, cache=self.cache)

    # Returns the list of augmenters of a single file
    def augment_single_file(self, orig_file):
        orig = self.get_orig_file(orig_file)
        augmented_list = [orig]

        for i in range(self.num_crops):
            augmented_list.append(AugCrop(orig))

        if self.add_mirror:
            aug_temp = augmented_list[:]

            for aug in aug_temp:
                augmented_list.append(AugMirrorH(aug))

        return augmented_list

    # Retruns the list of all files augmented
    def get_elems_augmented(self):
        list = []

        for file in self.files:
            list.extend(self.augment_single_file(file))

        return list

    def get_elems_augmented_split(self, validation_percentage, augment_validation=False):
        files_training, files_validation = train_test_split(self.files, test_size=validation_percentage, train_size=1 - validation_percentage)
        list_training = []
        list_validation = []

        for file in files_training:
            list_training.extend(self.augment_single_file(file))

        for file in files_validation:
            list_validation.extend(self.augment_single_file(file) if augment_validation else [self.get_orig_file(file)])

        return (list_training, list_validation)

# All the DataClasses
class DataSet:
    def __init__(self, data_classes, validation_percentage, augment_validation):
        self.data_classes = data_classes
        self.validation_percentage = validation_percentage
        self.train = []
        self.validation = []

        for single_class in data_classes:
            # items_single_class = single_class.get_elems_augmented()
            # train_single_class, test_single_class = train_test_split(items_single_class, test_size=validation_percentage, train_size=1-validation_percentage)
            train_single_class, test_single_class = single_class.get_elems_augmented_split(validation_percentage, augment_validation=augment_validation)

            self.train.extend(train_single_class)
            self.validation.extend(test_single_class)

    def get_training_len(self):
        return len(self.train)

    def get_validation_len(self):
        return len(self.validation)

    def get_validation_set(self, randomize=True):
        if randomize:
            return shuffle(self.validation)

        return self.validation

    def get_training_set(self, randomize=True):
        if randomize:
            return shuffle(self.train)
        return self.train

    def get_training_generator(self, batch_size=32):
        return self.generator(self.train, batch_size)

    def get_validation_generator(self, batch_size=32):
        return self.generator(self.validation, batch_size)

    def generator(self, augmented_samples, batch_size=32):
        num_samples = len(augmented_samples)
        while 1: # Loop forever so the generator never terminates
            augmented_samples = sklearn.utils.shuffle(augmented_samples)
            batch_partial_size = batch_size

            for offset in range(0, num_samples, batch_partial_size):
                batch_augmented_samples = augmented_samples[offset:offset + batch_partial_size]

                batch_samples = [x.get() for x in batch_augmented_samples]
                batch_labels = [x.label_hot for x in batch_augmented_samples]

                images = []
                outputs = []
                for idx in range(0, len(batch_samples)):
                    images.append(batch_samples[idx])
                    outputs.append(batch_labels[idx])

                X_train_ar = np.array(images)
                y_train_ar = np.array(outputs)

                yield sklearn.utils.shuffle(X_train_ar, y_train_ar)