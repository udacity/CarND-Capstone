from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os
import logging
import numpy as np
from matplotlib import pyplot as plt
import cv2
import pprint             #format data structures into strings, for logging
#MODEL_NAME = 'tld_simulator_frcnni_inference_graph'
#PATH_TO_LABELS = os.path.join('frcnni_simulator_training/', 'tld_simulator_label_map.pbtxt')
#NUM_CLASSES = 3
dir_path = os.path.dirname(os.path.realpath(__file__))
PATH_TO_CKPT = dir_path + '/models/tld_simulator_model/frozen_inference_graph.pb'
IMAGE_CAPTURE          = False
IMAGE_CAPTURE_PATH     = dir_path + '/captured_images'
DEBUG_MODE             = True #DEBUG_MODE does not send messages to terminal unless it is set in tl_detector.py

class TLClassifier(object):
    def __init__(self,debug=None,info=None,warn=None,error=None):
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

        self.debug("tl_classifier: using tensor flow version %s"%tf.__version__)
        if(DEBUG_MODE):
            self.warning("tl_classifier: DEBUG_MODE enabled.  Disable for PR or submission")
        if(IMAGE_CAPTURE):
            self.warning("tl_classifier: IMAGE_CAPTURE enabled.  Disable for PR or submission")

        self.img_count = 0
            
        #load classifier
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def run_inference_for_single_image(self,image, graph):
        output_dict = None
        with graph.as_default():
            with tf.Session() as sess:
                # Get handles to input and output tensors
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = {output.name for op in ops for output in op.outputs}
                tensor_dict = {}
                #for key in [
                #        'num_detections', 'detection_boxes', 'detection_scores',
                #        'detection_classes', 'detection_masks'
                #]:
                for key in [
                        'num_detections', 'detection_scores', 'detection_classes', 
                ]:
                    tensor_name = key + ':0'
                    if tensor_name in all_tensor_names:
                        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                            tensor_name)
                if 'detection_masks' in tensor_dict:
                    # The following processing is only for single image
                    detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
                    detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
                    # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
                    real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
                    detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
                    detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
                    detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                        detection_masks, detection_boxes, image.shape[0], image.shape[1])
                    detection_masks_reframed = tf.cast(
                        tf.greater(detection_masks_reframed, 0.5), tf.uint8)
                    # Follow the convention by adding back the batch dimension
                    tensor_dict['detection_masks'] = tf.expand_dims(
                        detection_masks_reframed, 0)

                image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
                image_expand = np.expand_dims(image, 0)
                #image_size = image_expand.shape
                #self.debug("image for inferencing has shape:")
                #self.pplog('debug',image_size)

                ### Run inference
                output_dict = sess.run(tensor_dict,
                                       feed_dict={image_tensor: image_expand})

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
        #TODO implement light color prediction      
        self.debug("tl_classifier: entered get_classification")
        if(DEBUG_MODE and IMAGE_CAPTURE):
            self.debug("tl_classifier: writing received image, numbered: %d"%self.img_count)
            if not os.path.exists(IMAGE_CAPTURE_PATH):
                self.debug("  creating directory %s"%IMAGE_CAPTURE_PATH)
                os.makedirs(IMAGE_CAPTURE_PATH)
            filename = IMAGE_CAPTURE_PATH + "/tc_image_%d.jpg"%self.img_count
            self.debug("  calling imwrite on file %s"%filename)
            cv2.imwrite(filename,cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
            self.img_count += 1

        #image_expanded = np.exapnd_dims(image,axis=0)
        output_dict = self.run_inference_for_single_image(image,self.detection_graph)
        self.debug("tl_classifier: inference returned the following output_dict:")
        self.pplog('debug',output_dict)

        return TrafficLight.UNKNOWN
