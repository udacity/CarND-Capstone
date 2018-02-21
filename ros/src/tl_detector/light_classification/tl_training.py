# -*- coding: utf-8 -*-
"""
Created on Wed Feb 21 08:16:48 2018

@author: Danilo Canivel
This need to be runned at the cli just when you want to generate a new trained model
Thanks for King Fengji, amazing library for Deep Forests, gcForest https://github.com/kingfengji/gcForest
"""

import argparse
import numpy as np
import sys
import pickle
import os
import cv2
import random as rand
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score

from gcforest.gcforest import GCForest
from gcforest.utils.config_utils import load_json

#Set to False to run only 1 ETC Fold for testing otherwise edit as you prefer at get_toy_config method
all_estimators = True

def get_files(dir_path):
    return [f for f in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, f))]

dataset_samples = []
dataset_labels = []

def read_dataset_dir(dir_path, label, dataset_samples, dataset_labels):
    for filename in get_files(dir_path):
        image = cv2.cvtColor(cv2.imread(os.path.join(dir_path, filename)), cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (24, 72))
        dataset_samples.append(image)
        dataset_labels.append(label)
        

red_samples, red_labels = [], []
read_dataset_dir('../../../../data/trafficlights/simulator/red', 0, red_samples, red_labels)
read_dataset_dir('../../../../data/trafficlights/udacity-sdc/red', 0, red_samples, red_labels)

yellow_samples, yellow_labels = [], []
read_dataset_dir('../../../../data/trafficlights/simulator/yellow', 1, yellow_samples, yellow_labels)
read_dataset_dir('../../../../data/trafficlights/udacity-sdc/yellow', 1, yellow_samples, yellow_labels)

green_samples, green_labels = [], []
read_dataset_dir('../../../../data/trafficlights/udacity-sdc/green', 2, green_samples, green_labels)
read_dataset_dir('../../../../data/trafficlights/simulator/green', 2, green_samples, green_labels)

red_samples = np.array(red_samples)
red_labels = np.array(red_labels)

X_red_train, X_red_test, y_red_train, y_red_test = train_test_split(
    red_samples,
    red_labels,
    test_size=0.05,
    random_state=42
)

X_yellow_train, X_yellow_test, y_yellow_train, y_yellow_test = train_test_split(
    yellow_samples,
    yellow_labels,
    test_size=0.05,
    random_state=42
)

X_green_train, X_green_test, y_green_train, y_green_test = train_test_split(
    green_samples,
    green_labels,
    test_size=0.05,
    random_state=42
)

X_train = np.concatenate([X_red_train, X_yellow_train, X_green_train], axis=0)
y_train = np.concatenate([y_red_train, y_yellow_train, y_green_train], axis=0)

X_test = np.concatenate([X_red_test, X_yellow_test, X_green_test], axis=0)
y_test = np.concatenate([y_red_test, y_yellow_test, y_green_test], axis=0)

n_train = X_train.shape [0]
n_test = X_test.shape [0]
image_shape = X_train.shape [1:3]
n_classes = 3

#rescaling of pixel magnitudes
X_train_preprocessed = np.divide(X_train, 255).astype (np.float32);
X_test_preprocessed = np.divide(X_test, 255).astype (np.float32);

n_train_classes = [np.count_nonzero(y_train==i) for i in range (n_classes)] 
n_test_classes = [np.count_nonzero(y_test==i) for i in range (n_classes)] 

#dividing into train and validation dataset
def shuffle_dataset (dataset_x, dataset_y):
    assert len (dataset_x) == len (dataset_y)
    p = np.random.permutation(len(dataset_x))
    return dataset_x [p], dataset_y [p]

def train_validation_split (dataset_x, dataset_y, train_proportion):
    train_samples_by_classes = []
    train_labels_by_classes = []
    X_train_samples = np.zeros ((0, image_shape[0], image_shape[1], 3))
    y_train_samples = np.zeros ((0,))
    X_validation_samples = np.zeros ((0, image_shape[0], image_shape[1], 3))
    y_validation_samples = np.zeros ((0,))
    sample_index = 0
    for class_size in n_train_classes:
        train_samples_count = int(train_proportion * class_size)

        #getting and shuffle one class samples
        dataset_x_for_class = dataset_x[sample_index:sample_index + class_size]
        dataset_y_for_class = dataset_y[sample_index:sample_index + class_size]
        dataset_x_for_class, dataset_y_for_class = shuffle_dataset (dataset_x_for_class, dataset_y_for_class)
        
        #splitting to train and validation dataset
        X_train_for_class = dataset_x_for_class[0:train_samples_count]
        y_train_for_class = dataset_y_for_class[0:train_samples_count]
        X_valid_for_class = dataset_x_for_class[train_samples_count:class_size]
        y_valid_for_class = dataset_y_for_class[train_samples_count:class_size]
        
        #storing train samples by classes
        #this will be used later to generate augmented dataset
        train_samples_by_classes.append (X_train_for_class)
        train_labels_by_classes.append (y_train_for_class)
        
        #storing samples in corresponding datasets
        X_train_samples = np.concatenate((X_train_samples, X_train_for_class), axis=0)
        y_train_samples = np.concatenate((y_train_samples, y_train_for_class), axis=0)
        X_validation_samples = np.concatenate((X_validation_samples, X_valid_for_class), axis=0)
        y_validation_samples = np.concatenate((y_validation_samples, y_valid_for_class), axis=0)
        
        sample_index += class_size
        
    return X_train_samples, y_train_samples, X_validation_samples, y_validation_samples, train_samples_by_classes, train_labels_by_classes

X_train_s, y_train_s, X_valid_s, y_valid_s, train_samples, labels_by_classes = train_validation_split (
    X_train_preprocessed, y_train, 0.8)

# generate @generate_count images from given @images
def augment_images (images, generate_count):
    images_last_index = len (images) - 1
    augmented = []
    for i in range (generate_count):
        im1 = images [rand.randint (0, images_last_index)]
        
        #rotation and scaling
        Mrot = cv2.getRotationMatrix2D((16,16),rand.uniform(-5.0, 5.0), rand.uniform(0.95, 1.05))

        #affine transform and shifts
        pts1 = np.float32([[0,0],[image_shape[1],0],[image_shape[1], image_shape[0]]])
        a = 5;
        shift = 8
        shiftx = rand.randint (-shift, shift);
        shifty = rand.randint (-shift, shift);
        pts2 = np.float32([[
                    0 + rand.randint (-a, a) + shiftx,
                    0 + rand.randint (-a, a) + shifty
                ],[
                    image_shape[1] + rand.randint (-a, a) + shiftx,
                    0 + rand.randint (-a, a) + shifty
                ],[
                    image_shape[1] + rand.randint (-a, a) + shiftx,
                    image_shape[0] + rand.randint (-a, a) + shifty
                ]])
        M = cv2.getAffineTransform(pts1,pts2)
        (h, w) = im1.shape[:2]

        augmented_image = cv2.warpAffine(
            cv2.warpAffine (
                im1
                , Mrot, (w, h)
            )
            , M, (w,h)
        )
        
        augmented_image += rand.uniform(-0.2, 0.2)
        np.clip(augmented_image, 0.0, 1.0, out=augmented_image)
        
        augmented.append (augmented_image)
        
    return augmented

#augmented dataset
X_train_augmented = np.zeros ((0, image_shape[0], image_shape[1], 3))
y_train_augmented = np.zeros ((0,))

#generate images up to 3000 images for each class
#augmented dataset will contain only generated images
augment_limit = 3000
def augment_dataset ():
    global X_train_augmented
    global y_train_augmented
    X_train_augmented = np.zeros ((0, image_shape[0], image_shape[1], 3))
    y_train_augmented = np.zeros ((0,))
    
    for augmenting_index in range(n_classes):
        samples = train_samples[augmenting_index]
        labels = labels_by_classes [augmenting_index]

        augment_count = augment_limit
        new_samples = augment_images (samples, augment_count)
        y_train_augmented = np.concatenate((y_train_augmented, [augmenting_index for i in range (augment_count)]), axis=0)

        X_train_augmented = np.concatenate((X_train_augmented, new_samples), axis=0)
    
augment_dataset ()

train_dataset_x, train_dataset_y = shuffle_dataset (X_train_augmented, y_train_augmented)

def get_toy_config(all_estimators=False):
    config = {}
    ca_config = {}
    ca_config["random_state"] = 0
    ca_config["max_layers"] = 100
    ca_config["early_stopping_rounds"] = 3
    ca_config["n_classes"] = n_classes
    ca_config["estimators"] = []
    if(all_estimators):
        ca_config["estimators"].append(
                  {"n_folds": 5, "type": "XGBClassifier", "n_estimators": 10, "max_depth": 5,
                   "objective": "multi:softprob", "silent": True, "nthread": -1, "learning_rate": 0.1} )
        ca_config["estimators"].append({"n_folds": 5, "type": "RandomForestClassifier", "n_estimators": 10, "max_depth": None, "n_jobs": -1})
        ca_config["estimators"].append({"n_folds": 5, "type": "ExtraTreesClassifier", "n_estimators": 10, "max_depth": None, "n_jobs": -1})
        ca_config["estimators"].append({"n_folds": 5, "type": "LogisticRegression"})
    else:
        ca_config["estimators"].append({"n_folds": 1, "type": "ExtraTreesClassifier", "n_estimators": 10, "max_depth": None, "n_jobs": -1})
    config["cascade"] = ca_config
    return config

config = get_toy_config(all_estimators=all_estimators)
gc = GCForest(config)
# If the model you use cost too much memory for you.
# You can use these methods to force gcforest not keeping model in memory
# gc.set_keep_model_in_mem(False), default is TRUE.
n_test = 500
# (X_train, y_train), (X_test, y_test) = mnist.load_data()
X_train, y_train = train_dataset_x[:-n_test], train_dataset_y[:-n_test]
X_test_cv, y_test_cv = train_dataset_x[-n_test:], train_dataset_y[-n_test:]

X_train = X_train[:, np.newaxis, :, :]
X_test_cv = X_test_cv[:, np.newaxis, :, :]

X_train_enc = gc.fit_transform(X_train, y_train)

y_pred_cv = gc.predict(X_test_cv)
acc = accuracy_score(y_test_cv, y_pred_cv)
print("Test Accuracy CV of GcForest = {:.2f} %".format(acc * 100))

y_pred = gc.predict(X_test_preprocessed)
acc = accuracy_score(y_test, y_pred)
print("Test Accuracy of GcForest = {:.2f} %".format(acc * 100))

# save the model to disk
with open("gc_classifier.pkl", "wb") as f:
    pickle.dump(gc, f, pickle.HIGHEST_PROTOCOL)