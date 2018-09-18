# Copyright 2017 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""Class Head.

Contains Class prediction head classes for different meta architectures.
All the class prediction heads have a predict function that receives the
`features` as the first argument and returns class predictions with background.
"""
import tensorflow as tf

from object_detection.predictors.heads import head

slim = tf.contrib.slim


class MaskRCNNClassHead(head.Head):
  """Mask RCNN class prediction head.

  Please refer to Mask RCNN paper:
  https://arxiv.org/abs/1703.06870
  """

  def __init__(self, is_training, num_classes, fc_hyperparams_fn,
               use_dropout, dropout_keep_prob):
    """Constructor.

    Args:
      is_training: Indicates whether the BoxPredictor is in training mode.
      num_classes: number of classes.  Note that num_classes *does not*
        include the background category, so if groundtruth labels take values
        in {0, 1, .., K-1}, num_classes=K (and not K+1, even though the
        assigned classification targets can range from {0,... K}).
      fc_hyperparams_fn: A function to generate tf-slim arg_scope with
        hyperparameters for fully connected ops.
      use_dropout: Option to use dropout or not.  Note that a single dropout
        op is applied here prior to both box and class predictions, which stands
        in contrast to the ConvolutionalBoxPredictor below.
      dropout_keep_prob: Keep probability for dropout.
        This is only used if use_dropout is True.
    """
    super(MaskRCNNClassHead, self).__init__()
    self._is_training = is_training
    self._num_classes = num_classes
    self._fc_hyperparams_fn = fc_hyperparams_fn
    self._use_dropout = use_dropout
    self._dropout_keep_prob = dropout_keep_prob

  def predict(self, features, num_predictions_per_location=1):
    """Predicts boxes and class scores.

    Args:
      features: A float tensor of shape [batch_size, height, width, channels]
        containing features for a batch of images.
      num_predictions_per_location: Int containing number of predictions per
        location.

    Returns:
      class_predictions_with_background: A float tensor of shape
        [batch_size, 1, num_classes + 1] representing the class predictions for
        the proposals.

    Raises:
      ValueError: If num_predictions_per_location is not 1.
    """
    if num_predictions_per_location != 1:
      raise ValueError('Only num_predictions_per_location=1 is supported')
    spatial_averaged_roi_pooled_features = tf.reduce_mean(
        features, [1, 2], keep_dims=True, name='AvgPool')
    flattened_roi_pooled_features = slim.flatten(
        spatial_averaged_roi_pooled_features)
    if self._use_dropout:
      flattened_roi_pooled_features = slim.dropout(
          flattened_roi_pooled_features,
          keep_prob=self._dropout_keep_prob,
          is_training=self._is_training)

    with slim.arg_scope(self._fc_hyperparams_fn()):
      class_predictions_with_background = slim.fully_connected(
          flattened_roi_pooled_features,
          self._num_classes + 1,
          activation_fn=None,
          scope='ClassPredictor')
    class_predictions_with_background = tf.reshape(
        class_predictions_with_background, [-1, 1, self._num_classes + 1])
    return class_predictions_with_background


class ConvolutionalClassHead(head.Head):
  """Convolutional class prediction head."""

  def __init__(self,
               is_training,
               num_classes,
               use_dropout,
               dropout_keep_prob,
               kernel_size,
               apply_sigmoid_to_scores=False,
               class_prediction_bias_init=0.0,
               use_depthwise=False):
    """Constructor.

    Args:
      is_training: Indicates whether the BoxPredictor is in training mode.
      num_classes: Number of classes.
      use_dropout: Option to use dropout or not.  Note that a single dropout
        op is applied here prior to both box and class predictions, which stands
        in contrast to the ConvolutionalBoxPredictor below.
      dropout_keep_prob: Keep probability for dropout.
        This is only used if use_dropout is True.
      kernel_size: Size of final convolution kernel.  If the
        spatial resolution of the feature map is smaller than the kernel size,
        then the kernel size is automatically set to be
        min(feature_width, feature_height).
      apply_sigmoid_to_scores: if True, apply the sigmoid on the output
        class_predictions.
      class_prediction_bias_init: constant value to initialize bias of the last
        conv2d layer before class prediction.
      use_depthwise: Whether to use depthwise convolutions for prediction
        steps. Default is False.

    Raises:
      ValueError: if min_depth > max_depth.
    """
    super(ConvolutionalClassHead, self).__init__()
    self._is_training = is_training
    self._num_classes = num_classes
    self._use_dropout = use_dropout
    self._dropout_keep_prob = dropout_keep_prob
    self._kernel_size = kernel_size
    self._apply_sigmoid_to_scores = apply_sigmoid_to_scores
    self._class_prediction_bias_init = class_prediction_bias_init
    self._use_depthwise = use_depthwise

  def predict(self, features, num_predictions_per_location):
    """Predicts boxes.

    Args:
      features: A float tensor of shape [batch_size, height, width, channels]
        containing image features.
      num_predictions_per_location: Number of box predictions to be made per
        spatial location.

    Returns:
      class_predictions_with_background: A float tensors of shape
        [batch_size, num_anchors, num_classes + 1] representing the class
        predictions for the proposals.
    """
    net = features
    # Add a slot for the background class.
    num_class_slots = self._num_classes + 1
    if self._use_dropout:
      net = slim.dropout(net, keep_prob=self._dropout_keep_prob)
    if self._use_depthwise:
      class_predictions_with_background = slim.separable_conv2d(
          net, None, [self._kernel_size, self._kernel_size],
          padding='SAME', depth_multiplier=1, stride=1,
          rate=1, scope='ClassPredictor_depthwise')
      class_predictions_with_background = slim.conv2d(
          class_predictions_with_background,
          num_predictions_per_location * num_class_slots, [1, 1],
          activation_fn=None,
          normalizer_fn=None,
          normalizer_params=None,
          scope='ClassPredictor')
    else:
      class_predictions_with_background = slim.conv2d(
          net,
          num_predictions_per_location * num_class_slots,
          [self._kernel_size, self._kernel_size],
          activation_fn=None,
          normalizer_fn=None,
          normalizer_params=None,
          scope='ClassPredictor',
          biases_initializer=tf.constant_initializer(
              self._class_prediction_bias_init))
    if self._apply_sigmoid_to_scores:
      class_predictions_with_background = tf.sigmoid(
          class_predictions_with_background)
    batch_size = features.get_shape().as_list()[0]
    if batch_size is None:
      batch_size = tf.shape(features)[0]
    class_predictions_with_background = tf.reshape(
        class_predictions_with_background, [batch_size, -1, num_class_slots])
    return class_predictions_with_background


# TODO(alirezafathi): See if possible to unify Weight Shared with regular
# convolutional class head.
class WeightSharedConvolutionalClassHead(head.Head):
  """Weight shared convolutional class prediction head.

  This head allows sharing the same set of parameters (weights) when called more
  then once on different feature maps.
  """

  def __init__(self,
               num_classes,
               kernel_size=3,
               class_prediction_bias_init=0.0,
               use_dropout=False,
               dropout_keep_prob=0.8):
    """Constructor.

    Args:
      num_classes: number of classes.  Note that num_classes *does not*
        include the background category, so if groundtruth labels take values
        in {0, 1, .., K-1}, num_classes=K (and not K+1, even though the
        assigned classification targets can range from {0,... K}).
      kernel_size: Size of final convolution kernel.
      class_prediction_bias_init: constant value to initialize bias of the last
        conv2d layer before class prediction.
      use_dropout: Whether to apply dropout to class prediction head.
      dropout_keep_prob: Probability of keeping activiations.
    """
    super(WeightSharedConvolutionalClassHead, self).__init__()
    self._num_classes = num_classes
    self._kernel_size = kernel_size
    self._class_prediction_bias_init = class_prediction_bias_init
    self._use_dropout = use_dropout
    self._dropout_keep_prob = dropout_keep_prob

  def predict(self, features, num_predictions_per_location):
    """Predicts boxes.

    Args:
      features: A float tensor of shape [batch_size, height, width, channels]
        containing image features.
      num_predictions_per_location: Number of box predictions to be made per
        spatial location.

    Returns:
      class_predictions_with_background: A tensor of shape
        [batch_size, num_anchors, num_classes + 1] representing the class
        predictions for the proposals.
    """
    class_predictions_net = features
    num_class_slots = self._num_classes + 1
    # Add a slot for the background class.
    if self._use_dropout:
      class_predictions_net = slim.dropout(
          class_predictions_net, keep_prob=self._dropout_keep_prob)
    class_predictions_with_background = slim.conv2d(
        class_predictions_net,
        num_predictions_per_location * num_class_slots,
        [self._kernel_size, self._kernel_size],
        activation_fn=None, stride=1, padding='SAME',
        normalizer_fn=None,
        biases_initializer=tf.constant_initializer(
            self._class_prediction_bias_init),
        scope='ClassPredictor')
    batch_size = features.get_shape().as_list()[0]
    if batch_size is None:
      batch_size = tf.shape(features)[0]
    class_predictions_with_background = tf.reshape(
        class_predictions_with_background, [batch_size, -1, num_class_slots])
    return class_predictions_with_background
