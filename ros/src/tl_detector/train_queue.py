import cv2
from Queue import Queue
from threading import Thread
from styx_msgs.msg import TrafficLightArray, TrafficLight

class TrainItem:
    def __init__(self, state, behind, distance, cv_image):
        self.state = state
        self.behind = behind
        self.distance = distance
        self.cv_image = cv_image

def friendly_name(state):
    state_name_dict = {TrafficLight.UNKNOWN: "Unknown",
                       TrafficLight.RED: "Red",
                       TrafficLight.YELLOW: "Red",
                       TrafficLight.GREEN : "Green"}
    return state_name_dict[state]

class TrainQueue:
    def __init__(self):
        self.imgidx = 1322
        self.q = Queue()
        self.t = Thread(target=self.process_queue)
        self.t.start()

    def enqueue(self, item):
        self.q.put(item)

    def process_queue(self):
        while True:
            item = self.q.get(block = True)

            if False:
                cv2.putText(item.cv_image, friendly_name(item.state), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 5)
                cv2.putText(item.cv_image, 'behind: {0}'.format(item.behind), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 5)
                cv2.putText(item.cv_image, 'distance: {0}'.format(item.distance), (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 5)

            cv2.imwrite('./out/dev/{:06d}.png'.format(self.imgidx), item.cv_image)
            self.imgidx += 1
