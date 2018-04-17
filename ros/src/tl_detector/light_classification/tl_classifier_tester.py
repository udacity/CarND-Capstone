import os
import cv2
from tl_classifier import TLClassifier

model = 'light_classification/training/train_ssd_site/fine_tuned_model/frozen_inference_graph.pb'
label_path = 'light_classification/training/site_training_data/label_map.pbtxt'
# model = 'light_classification/mmsarode_frozen_inference_graph.pb'
# label_path = 'light_classification/mmsarode_label_map.pbtxt'

num_classes = 4
light_classifier = TLClassifier(model, label_path, num_classes)


test_images_dir = "light_classification/training/site_training_data"
result_images_dir = "light_classification/results"
for filename in os.listdir(test_images_dir):
    if filename.endswith(".jpg"):
        cv_image = cv2.imread(os.path.join(test_images_dir, filename))
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image, classes, dt = light_classifier.get_classification(cv_image)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(os.path.join(result_images_dir, filename), image)
        print(classes)
