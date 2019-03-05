from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        if tf.__version__ < '1.4.0':
            rospy.logwarn('Please upgrade your tensorflow installation to v1.4.* or later! you have version '+tf.__version__ )
        PATH_TO_CKPT = 'light_classification/model/frozen_inference_graph.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        image_np = self.load_image_into_numpy_array(image)
        # Actual detection.
        output_dict = self.run_inference_for_single_image(image_np, self.detection_graph)
        text_string = "Classified light state : {0} with probability {1}"
        if (output_dict['detection_scores'][0] > 0.5):
            text_string += " > 0.5"
        else:
            text_string += " <= 0.5"
        rospy.logwarn(text_string.format(output_dict['detection_classes'][0],output_dict['detection_scores'][0]))
        #print(output_dict['detection_boxes'][0]) # if we go for the 2 step approach
        if (output_dict['detection_scores'][0] > 0.5):
            """
            # Base model assignment
            if (output_dict['detection_classes'][0] == 3 ):
                return TrafficLight.GREEN
            if (output_dict['detection_classes'][0] == 2 ):
                return TrafficLight.YELLOW
            if (output_dict['detection_classes'][0] == 1 ):
                return TrafficLight.RED
            """
            # New model assignment
            if (output_dict['detection_classes'][0] == 1 ):
                return TrafficLight.GREEN
            if (output_dict['detection_classes'][0] == 2 ):
                return TrafficLight.YELLOW
            if (output_dict['detection_classes'][0] == 3 ):
                return TrafficLight.RED

        return TrafficLight.UNKNOWN

    def load_image_into_numpy_array(self,image):
        im_height, im_width = image.shape[:2]
        #(im_width, im_height) = image.size
        return image.reshape((im_height, im_width, 3)).astype(np.uint8)

    def run_inference_for_single_image(self,image, graph):
        with graph.as_default():
            with tf.Session() as sess:
                # Get handles to input and output tensors
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = {output.name for op in ops for output in op.outputs}
                tensor_dict = {}
                for key in [
                    'num_detections', 'detection_boxes', 'detection_scores',
                    'detection_classes', 'detection_masks'
                ]:
                    tensor_name = key + ':0'
                    if tensor_name in all_tensor_names:
                        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)

                image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

                # Run inference
                output_dict = sess.run(tensor_dict,
                                        feed_dict={image_tensor: np.expand_dims(image, 0)})

                # all outputs are float32 numpy arrays, so convert types as appropriate
                output_dict['num_detections'] = int(output_dict['num_detections'][0])
                output_dict['detection_classes'] = output_dict[
                    'detection_classes'][0].astype(np.uint8)
                output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
                output_dict['detection_scores'] = output_dict['detection_scores'][0]

        return output_dict
