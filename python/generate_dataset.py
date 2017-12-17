import os
import csv
import cv2
from os.path import isfile, join

def read_labels(file_csv):
    list = []
    with open(file_csv, 'r') as f:
        reader = csv.reader(f, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
        for row in reader:
            list.append(row)
        # i need to strip the extra white space from each string in the row
        return (list)

FILE = 0
X1 = 1
Y1 = 2
X2 = 3
Y2 = 4
OCCLUDED = 5
LABEL = 6
ATTRIBUTES = 7

def crop_and_resize(labels, origin, destRgb, destGray):
    n = 0
    print(destRgb)
    print(destGray)

    for l in labels:
        img = cv2.imread(origin+l[FILE])
        y1 = int(l[Y1])
        y2 = int(l[Y2])
        x1 = int(l[X1])
        x2 = int(l[X2])

        roiRgb = img[y1:y2, x1:x2]
        roiGray = cv2.cvtColor(roiRgb, cv2.COLOR_BGR2GRAY)

        nameRgb = destRgb + l[ATTRIBUTES] + "\\" + str(n) + ".jpg"
        nameGray = destGray + l[ATTRIBUTES] + "\\" + str(n) + ".jpg"
        #print(nameRgb)
        #print(nameGray)
        cv2.imwrite(nameRgb, roiRgb)
        cv2.imwrite(nameGray, roiGray)
        n += 1

        if n%100 == 0:
            print("Image", n, "ouf ot", len(labels))

origin = "C:\\datasets\\Udacity\\object-dataset\\"
#files = files_from(origin, include_sub_directories=False)
#files = [f for f in files if f.endswith(".jpg")]
labels = read_labels(origin + "labels.csv")
labels_lights = [l for l in labels if l[LABEL] == "trafficLight"]
labels_lights_att = [l for l in labels_lights if (len(l)>ATTRIBUTES) and l[ATTRIBUTES]]

print("Total files:", len(labels))
print("Frames annotated with Traffic lights:", len(labels_lights))
print("Frames annotated with Traffic lights and attributes:", len(labels_lights_att))

crop_and_resize(labels_lights_att, origin, origin + "outRgb\\", origin + "outGray\\")

