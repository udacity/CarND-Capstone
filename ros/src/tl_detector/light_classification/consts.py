# Author: Guy Hadash
# Source: https://github.com/mynameisguy/TrafficLightChallenge-DeepLearning-Nexar

## dirs confs
DATASET_FOLDER="/home/eljefec/data/traffic_light_bag_files/rgb"
MODELS_CHECKPOINTS_DIR="./checkpoints"

# training confs
BATCH_SIZE = 64
TRAINING_EPOCHS = 200 #max
# TRAIN_IMAGES_PER_EPOCH = 1677
TRAIN_IMAGES_PER_EPOCH = 1950
# VALIDATE_IMAGES_PER_EPOCH = 186
VALIDATE_IMAGES_PER_EPOCH = 487
IMAGE_WIDTH = 224
IMAGE_HEIGHT = 224
