'''
Resources:
    - https://github.com/ulmefors/CarND-Capstone/blob/vpcom/classifiers/WholePicDL/trafficLightClassifer.ipynb
    - https://datascience.stackexchange.com/questions/13490/how-to-set-class-weights-for-imbalanced-classes-in-keras
    - https://stackoverflow.com/questions/44274701/make-predictions-using-a-tensorflow-graph-from-a-keras-model
    - https://github.com/burgalon/deep-learning-traffic-lights/blob/master/train.py
    - https://www.tensorflow.org
'''
from styx_msgs.msg import TrafficLight
from keras.models import load_model
import tensorflow as tf
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.model = load_model('light_classification/03-val_acc-1.00.hdf5')
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()


    def load_graph(self, frozen_graph_filename):
	    # We load the protobuf file from the disk and parse it to retrieve the 
	    # unserialized graph_def
	    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
		graph_def = tf.GraphDef()
		graph_def.ParseFromString(f.read())

	    # Then, we can use again a convenient built-in function to import a graph_def into the 
	    # current default Graph
	    with tf.Graph().as_default() as graph:
		tf.import_graph_def(
		    graph_def, 
		    input_map=None, 
		    return_elements=None, 
		    name="prefix", 
		    op_dict=None, 
		    producer_op_list=None
		)
	    dropout_name = graph.get_operations()[0].name+':0'
	    input_name = graph.get_operations()[1].name+':0'
	    output_name = graph.get_operations()[-1].name+':0'

	    return graph, input_name, output_name, dropout_name
    def predict(self, model_path, input_data):
	    # load tf graph
	    tf_model,tf_input,tf_output, dropout_name = self.load_graph(model_path)

	    # Create tensors for model input and output
	    x = tf_model.get_tensor_by_name(tf_input)
	    y = tf_model.get_tensor_by_name(tf_output) 

	    # Number of model outputs
	    num_outputs = y.shape.as_list()[0]
	    print('Number of outputs: ', num_outputs)
	    predictions = np.zeros((input_data.shape[0],num_outputs))
	    for i in range(input_data.shape[0]):        
		with tf.Session(graph=tf_model) as sess:
		    #print(tf_model.get_operations()[1].name+':0')
		    y_out = sess.run(y, feed_dict={x: input_data[i:i+1], dropout_name: 0})
		    predictions[i] = y_out

	    return predictions
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	tl_class = TrafficLight.UNKNOWN
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (224, 224))/255.0
        input2tfModel = np.expand_dims(image, axis=0)

	# TODO fix the u'RealDiv' problem
	tf_model_path = 'light_classification/03-val_acc-1.00.hdpb'
	pred = self.predict(tf_model_path,input2tfModel)

	# use keras instead of TensorFlow
	#with self.graph.as_default():
            #pred = self.model.predict(input2tfModel)

	
	tf_status = np.argmax(pred)
	print('Prediction is', pred, ' TF Status: ', tf_status)
	if tf_status == 0:
		tl_class = TrafficLight.GREEN
	elif tf_status == 1: 
		tl_class = TrafficLight.RED
        #TODO implement light color prediction
	#status = #TrafficLight.GREEN // TrafficLight.RED // TrafficLight.YELLOW // TrafficLight.UNKNOWN
	
        return tl_class#TrafficLight.UNKNOWN
