import tensorflow as tf
from consts import *
from train import SqueezeNet

def save_model(sim):
    model_name_dict = {True: "squeezeNet_sim",
                       False: "squeezeNet_real"}

    model_name = model_name_dict[sim]

    with tf.Session() as sess:
        # Load model from https://github.com/mynameisguy/TrafficLightChallenge-DeepLearning-Nexar
        model = SqueezeNet(3, (IMAGE_HEIGHT, IMAGE_WIDTH, 3))
        model.load_weights("trained_model/{}.hdf5".format(model_name))
        init_op = tf.global_variables_initializer()
        sess.run(init_op)
        saver = tf.train.Saver().save(sess, '{}.ckpt'.format(model_name))
        graph_def = sess.graph.as_graph_def()
        tf.train.write_graph(graph_def, logdir='.', name='{}.pb'.format(model_name), as_text=False)
        tf.train.write_graph(graph_def, logdir='.', name='{}.pbtxt'.format(model_name), as_text=True)

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

def look_at_ops(graph_file):
    sess, ops = load_graph(graph_file)
    print(graph_file, len(ops))

def count_ops():
    look_at_ops('squeezeNet_real.pb')
    look_at_ops('squeezeNet_real.optimized.pb')
    look_at_ops('squeezeNet_sim.pb')
    look_at_ops('squeezeNet_sim.optimized.pb')

def save_models():
    save_model(False)
    save_model(True)

if __name__ == '__main__':
    save_models()
    count_ops()
