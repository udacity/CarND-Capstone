from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os
import logging
import numpy as np
import cv2
import pprint             #format data structures into strings, for logging
#MODEL_NAME = 'tld_simulator_frcnni_inference_graph'
#PATH_TO_LABELS = os.path.join('frcnni_simulator_training/', 'tld_simulator_label_map.pbtxt')
#NUM_CLASSES = 3

dir_path = os.path.dirname(os.path.realpath(__file__))

SIMULATOR_TRACK        = True  #Controls whether to use simulator track model instead of parking_lot
SCORE_THRESH           = 0.50  #detection_score threshold to report a positive result, or invalidate a differeing result
IMAGE_CAPTURE          = False #write images to file in debug mode.  Aside from initial work, doesn't make sense to enable until we add code to trigger on an incorrect result
IMAGE_CAPTURE_PATH     = dir_path + '/captured_images'
DEBUG_MODE             = False #DEBUG_MODE does not send messages to terminal unless it is set in tl_detector.py

PATH_TO_CKPT           = dir_path + '/models/tld_parking_lot_model/frozen_inference_graph.pb'
if (SIMULATOR_TRACK):
    PATH_TO_CKPT       = dir_path + '/models/tld_simulator_model/frozen_inference_graph.pb'

class TLClassifier(object):
    def __init__(self,image_size,debug=None,info=None,warn=None,error=None):
        self.debug = logging.debug
        self.info  = logging.info
        self.warning  = logging.warning
        self.error = logging.error
        if(debug):
            self.debug = debug
        if(info):
            self.info = info
        if(warn):
            self.warning = warn
        if(error):
            self.error = error

        if(DEBUG_MODE):
            self.warning("tl_classifier: DEBUG_MODE enabled.  Disable for PR or submission")
        if(IMAGE_CAPTURE):
            self.warning("tl_classifier: IMAGE_CAPTURE enabled.  Disable for PR or submission")
        if(SIMULATOR_TRACK):
            self.warning("tl_classifier: SIMULATOR_TRACK enabled.  Disable for submission")

        self.debug("tl_classifier: using tensor flow version %s"%tf.__version__)

        self.img_count = 0
            
        #load classifier
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        #first steps of inference moved here, so as never to repeat
            with tf.Session() as self.sess:
                # Get handles to input and output tensors
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = {output.name for op in ops for output in op.outputs}
                self.tensor_dict = {}
                #for key in [
                #        'num_detections', 'detection_boxes', 'detection_scores',
                #        'detection_classes', 'detection_masks'
                #]:
                for key in [
                        'num_detections', 'detection_scores', 'detection_classes', 
                ]:
                    tensor_name = key + ':0'
                    if tensor_name in all_tensor_names:
                        self.tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                            tensor_name)
                if 'detection_masks' in self.tensor_dict:
                    # The following processing is only for single image
                    detection_boxes = tf.squeeze(self.tensor_dict['detection_boxes'], [0])
                    detection_masks = tf.squeeze(self.tensor_dict['detection_masks'], [0])
                    # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
                    real_num_detection = tf.cast(self.tensor_dict['num_detections'][0], tf.int32)
                    detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
                    detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
                    detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                        detection_masks, detection_boxes, image_size[0], image_size[1])
                    detection_masks_reframed = tf.cast(
                        tf.greater(detection_masks_reframed, 0.5), tf.uint8)
                    # Follow the convention by adding back the batch dimension
                    self.tensor_dict['detection_masks'] = tf.expand_dims(
                        detection_masks_reframed, 0)
        
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

    def run_inference_for_single_image(self,image):

        image_expand = np.expand_dims(image, 0)
        #image_size = image_expand.shape
        #self.debug("image for inferencing has shape:")
        #self.pplog('debug',image_size)
        
        ### Run inference
        output_dict = self.sess.run(self.tensor_dict,
                                    feed_dict={self.image_tensor: image_expand})

        # all outputs are float32 numpy arrays, so convert types as appropriate
        for topkey in ('num_detections','detection_classes','detection_boxes','detection_scores','detection_masks'):
            if topkey in output_dict:
                if topkey == 'detection_classes':
                    output_dict[topkey] = output_dict[topkey][0].astype(np.uint8)
                else:
                    output_dict[topkey] = output_dict[topkey][0]

        return output_dict

    #def load_image_into_numpy_array(self,image):
    #    (im_width, im_height) = image.size
    #    return np.array(image.getdata()).reshape(
    #        (im_height, im_width, 3)).astype(np.uint8)
    
    def pplog(self,level,data): #generate string from prettyPrinter, and send to rospy.log<level>
        ppstring = pprint.pformat(data,indent=4)
        if (level == 'debug'):
            self.debug("   -- ppdata debug: --")
            self.debug(ppstring)
            self.debug("   -- end pp data debug --")
            return
        if (level == 'info'):
            self.info("   -- ppdata info: --")
            self.info(ppstring)
            self.info("   -- end pp data info --")
            return
        if (level == 'warn'):
            self.warning("   -- ppdata warn: --")
            self.warning(ppstring)
            self.warning("   -- end pp data warn --")
            return
        if (level == 'err'):
            self.error("   -- ppdata err: --")
            self.error(ppstring)
            self.error("   -- end pp data err --")
            return
        if (level == 'fatal'):
            self.fatal("   -- ppdata fatal: --")
            self.fatal(ppstring)
            self.fatal("   -- end pp data fatal --")
            return
        self.warning("tl_detector: pplog received unrecognized level: %s",level)
        return

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #DONE: implement light color prediction      
        self.img_count += 1
        self.debug("tl_classifier: entered get_classification for image %d",self.img_count)
        cur_image_num = self.img_count

        #image_expanded = np.exapnd_dims(image,axis=0)
        output_dict = self.run_inference_for_single_image(image)
        self.debug("tl_classifier: inference returned the following output_dict for image %d:",cur_image_num)
        self.pplog('debug',output_dict)
        class_state = -1 #used to represent 'unknown'
        light_state = TrafficLight.UNKNOWN
        light_string = "unknown"
        light_score  = 0
        #report a detection with score > SCORE_THRESH, unless another detection with score > SCORE_THRESH disagrees with it
        for i in range(output_dict['num_detections']):
            if(light_score <= 0):
                light_score = output_dict['detection_scores'][i]
            if(output_dict['detection_scores'][i] > SCORE_THRESH):
                if (class_state == -1):
                    class_state = output_dict['detection_classes'][i]
                else:
                    if(output_dict['detection_classes'][i] != class_state):
                        class_state = -1
                        break
                    
        if(class_state == 1):
            light_state = TrafficLight.RED
            light_string = 'red'
        elif(class_state == 2):
            light_state = TrafficLight.GREEN
            light_string = 'green'
        elif(class_state == 3):
            light_state = TrafficLight.YELLOW
            light_string = 'amber'

        if(DEBUG_MODE and IMAGE_CAPTURE):
            self.debug("tl_classifier: writing received image, numbered: %d"%cur_image_num)
            if not os.path.exists(IMAGE_CAPTURE_PATH):
                self.debug("  creating directory %s"%IMAGE_CAPTURE_PATH)
                os.makedirs(IMAGE_CAPTURE_PATH)
            filename = IMAGE_CAPTURE_PATH + "/tc_image_%d_%s_%.3f.jpg"%(cur_image_num,light_string,light_score)
            self.debug("  calling imwrite on file %s"%filename)
            cv2.imwrite(filename,cv2.cvtColor(image,cv2.COLOR_RGB2BGR))

        self.debug("tl_classifier: returning light_state %d for image %d",light_state,cur_image_num)
        return light_state
