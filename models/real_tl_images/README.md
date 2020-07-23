# Data

images.tgz

images/{Red,Yellow,Green,None}/${UNIQ_ID}_${BBOX_ID}_${CLASS_ID}.jpg

CLASS_ID: Red(1), Yellow(2 or 3), Green(4), None(0)

# Data sources

I collected real traffic light images from two sources.

## DriveU Traffic Light Dataset (DTLD)
- https://www.uni-ulm.de/en/in/driveu/projects/driveu-traffic-light-dataset/

You need to download this dataset after registration.
I downloaded only Bochum and Bremen because data size for other areas are too big to download.

## dataset-sdcnd-capstone.zip
- https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view

You can download this dataset quickly without any registration.
However, real traffic light images in this data set are too simple to generalize a ML model.

# Steps to create dataset

```
$ ./docker/build.sh
$ ./docker/start.sh
$ docker exec -it tl_image_collection /bin/bash
(root) ./setup.sh
(root) ./run.sh
```
