from squeezenet_tf import *
import tensorflow as tf
from tensorflow import layers
from glob import glob
from os import path
import argparse

def load_imgs_labels(img_dir, label2class):
    img_files = np.array(glob(path.join(img_dir, "*/*.jpg")))
    imgs = np.array([imread_resize(f)[0] for f in img_files])
    labels = [f.rsplit("/", 2)[-2] for f in img_files]
    targets = np.array([label2class[label] for label in labels])
    return imgs, targets

def make_tl_generator(imgs, targets, batch_size=128):
    """Traffic light images and labels generator
    """
    index = np.arange(imgs.shape[0])
    while True:
        np.random.shuffle(index)
        choice = index[:batch_size]
        x = imgs[choice]
        y = targets[choice]
        yield x, y

def main():

  # parsing
  parser = argparse.ArgumentParser()
  parser.add_argument("input", help="pretrained sqznet weight file, e.g., sqz_full.mat")
  parser.add_argument("output", help="folder for generated model")
  parser.add_argument("train", help="train folder dir with images in different subfolders")
  parser.add_argument("--test", help="optional test folder")
  args = parser.parse_args()

  # data loading
  label2class = {"green": 0, "yellow": 1, "red": 2, "nolight": 3}
  train_dir = args.train
  test_dir = args.test or args.train
  train_imgs, train_targets = load_imgs_labels(train_dir, label2class)
  print("loaded training images", train_imgs.shape)
  if test_dir != train_dir:
    test_imgs, test_targets = load_imgs_labels(test_dir, label2class)
  else:
    test_imgs, test_targets = train_imgs, train_targets
  print("loaded test images", test_imgs.shape)

  ## model
  # load weights as numpy array
  sqz_wts, sqz_mean = load_net(args.input)
  params = {"dropout_rate": 0.5, "n_classes":4, "learning_rate":5e-4}

  tf.reset_default_graph()

  image = tf.placeholder(tf.float32, shape=[None, 227, 227, 3])
  labels = tf.placeholder(tf.int32, shape=[None])
  training = tf.placeholder(tf.bool, shape=[])

  sqz_net = net_preloaded(sqz_wts, image, 'max', False) # weights as tf.constant
  bottleneck_feats = sqz_net['fire9/concat_conc'] # ?x13x13x512
  output = layers.dropout(bottleneck_feats, rate=params["dropout_rate"], training=training)
  output = layers.conv2d(output, params["n_classes"], kernel_size=[13, 13], strides=[1, 1])
  logits = tf.reshape(output, [-1, 4])
  predictions = tf.argmax(logits, axis=1)

  loss = tf.reduce_mean(tf.losses.sparse_softmax_cross_entropy(labels, logits))
  match = tf.nn.in_top_k(logits, labels, 1)
  accuracy = tf.reduce_mean(tf.cast(match, tf.float32))

  optimizer = tf.train.AdamOptimizer(learning_rate=params["learning_rate"])
  train_op = optimizer.minimize(loss, global_step=tf.train.get_global_step())

  saver = tf.train.Saver()
  tf.add_to_collection("model_input", image)
  tf.add_to_collection("model_input", labels)
  tf.add_to_collection("model_input", training)
  tf.add_to_collection("model_output", logits)
  tf.add_to_collection("model_output", predictions)

  ## training
  batch_size = 256

  n_epochs = train_imgs.shape[0] // batch_size * 10

  train_batches = make_tl_generator(train_imgs, train_targets, batch_size=batch_size)
  test_batches = make_tl_generator(test_imgs, test_targets, batch_size=batch_size)

  with tf.Session() as sess:
      sess.run(tf.global_variables_initializer())
      for epoch in range(n_epochs):
          x_batch, y_batch = next(train_batches)
          _, loss_val = sess.run([train_op, loss],
                                 feed_dict={
                                     image: x_batch,
                                     labels: y_batch,
                                     training: True
                                 })
          if epoch % 50 == 0:
              print(epoch, loss_val)
      model_file = saver.save(sess, path.join(args.output,"tl_model"))
      print("model saved to", model_file)
      ## test accuracy
      test_preds, test_accuracy = [], []
      for i in range(0, test_imgs.shape[0], batch_size):
          tp, ta = sess.run([predictions, accuracy],
                             feed_dict={
                                 image: test_imgs[i:i+batch_size],
                                 labels: test_targets[i:i+batch_size],
                                 training:False
                             })
          test_preds.append(tp)
          test_accuracy.append(ta)
  test_yhat = np.concatenate(test_preds)
  test_accuracy = np.mean(test_yhat==test_targets)
  print("test accuracy:", test_accuracy)

if __name__ == "__main__":
  main()