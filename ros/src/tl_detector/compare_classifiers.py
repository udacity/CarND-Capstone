import cv2
import os
from styx_msgs.msg import TrafficLight
import light_classification.image
import light_classification.tl_classifier
import light_classification.tl_classifier_opencv

def measure_accuracy(classes_filename, image_dir, classifier):
    correct = 0
    examples = light_classification.image.get_examples(classes_filename)
    for ex in examples:
        fullname = os.path.join(image_dir, ex.filename)
        img = cv2.imread(fullname)
        pred = classifier.get_classification(img)
        if pred == TrafficLight.YELLOW:
            # opencv can classify yellow, but training labels convert yellow to red
            pred = TrafficLight.RED
        if pred == ex.state:
            correct += 1
    return float(correct) / len(examples)

def compare_classifiers(sim):
    sim_classes = 'light_classification/sim/sorted_classes.txt'
    bag_classes = 'light_classification/bag/sorted_classes.txt'
    sim_image_dir = '/home/eljefec/data/traffic_light_sim/rgb'
    bag_image_dir = '/home/eljefec/data/traffic_light_bag_files/rgb'

    nn_classifier = light_classification.tl_classifier.TLClassifier(sim)
    opencv_classifier = light_classification.tl_classifier_opencv.TLClassifier()

    print('nn_classifier sim', sim)
    for classes_filename, image_dir in [(sim_classes, sim_image_dir), (bag_classes, bag_image_dir)]:
        print('image_dir', image_dir)
        for name, classifier in [('nn', nn_classifier), ('opencv', opencv_classifier)]:
            accuracy = measure_accuracy(classes_filename, image_dir, classifier)
            print(name, 'accuracy', accuracy)

if __name__ == '__main__':
    # compare_classifiers(True)
    compare_classifiers(False)
