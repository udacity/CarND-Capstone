import tensorflow as tf
from tensorflow.python.framework import graph_util
from consts import *
from train import SqueezeNet
import keras.backend as K

K.set_image_dim_ordering('tf')

model_name_dict = {True: "squeezeNet_sim",
                   False: "squeezeNet_real"}

def freeze_squeezenet(sim):
    model_name = model_name_dict[sim]
    model = SqueezeNet(3, (IMAGE_HEIGHT, IMAGE_WIDTH, 3))
    model.load_weights("trained_model/{}.hdf5".format(model_name))

    sess = K.get_session()

    graph = sess.graph
    input_graph_def = graph.as_graph_def()

    with sess.as_default():
        output_node_names = "softmax/Softmax"
        output_graph_def = graph_util.convert_variables_to_constants(
            sess, # The session is used to retrieve the weights
            input_graph_def, # The graph_def is used to retrieve the nodes
            output_node_names.split(",") # The output node names are used to select the usefull nodes
        )
        with tf.gfile.GFile('trained_model/{}.frozen.pb'.format(model_name), "wb") as f:
            f.write(output_graph_def.SerializeToString())
        print("%d ops in the final graph." % len(output_graph_def.node))

def load_graph(graph_file, use_xla=False):
    jit_level = 0
    config = tf.ConfigProto()
    if use_xla:
        jit_level = tf.OptimizerOptions.ON_1
        config.graph_options.optimizer_options.global_jit_level = jit_level

    with tf.Session(graph=tf.Graph(), config=config) as sess:
        gd = tf.GraphDef()
        with tf.gfile.Open(graph_file, 'rb') as f:
            data = f.read()
            gd.ParseFromString(data)
        tf.import_graph_def(gd, name='')
        ops = sess.graph.get_operations()
        n_ops = len(ops)
        return sess.graph, ops

def freeze():
    freeze_squeezenet(True)
    freeze_squeezenet(False)

if __name__ == '__main__':
    freeze()
