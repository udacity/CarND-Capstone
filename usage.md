# Usage

Convertion between rosimg to cv2
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

```python
class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

    def callback(self,data):
        try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
        print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
        cv2.circle(cv_image, (50,50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

```

```
rosrun image_view image_view image:=/tl_detection
```