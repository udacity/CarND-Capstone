# Scripts and methodology based on https://www.tensorflow.org/tutorials/image_retraining
#
# To run this script, you need to have your dataset in image_dir (ie tf_files/dataset).
# Your dataset must contain one directory per class and be named after the class.
#
# You can visualize training by performing:
#     tensorboard --logdir tf_files/training_summaries/ &

IMAGE_SIZE=128
ARCHITECTURE="mobilenet_0.25_${IMAGE_SIZE}"

python retrain.py \
  --bottleneck_dir=tf_files/bottlenecks \
  --how_many_training_steps=1000 \
  --model_dir=tf_files/models/ \
  --summaries_dir=tf_files/training_summaries/"${ARCHITECTURE}" \
  --output_graph=tf_files/trained_models/"${ARCHITECTURE}"/classifier_graph.pb \
  --output_labels=tf_files/trained_models/"${ARCHITECTURE}"/classifier_labels.txt \
  --architecture="${ARCHITECTURE}" \
  --image_dir=tf_files/dataset \
  --random_crop=10 \
  --random_scale=10 \
  --random_brightness=10 \
  --train_batch_size=1000 \
  --validation_batch_size=1000 \
  --eval_step_interval=10
#  --print_misclassified_test_images



IMAGE_SIZE=224
ARCHITECTURE="mobilenet_1.0_${IMAGE_SIZE}"

python retrain.py \
  --bottleneck_dir=tf_files/bottlenecks \
  --how_many_training_steps=1000 \
  --model_dir=tf_files/models/ \
  --summaries_dir=tf_files/training_summaries/"${ARCHITECTURE}" \
  --output_graph=tf_files/trained_models/"${ARCHITECTURE}"/classifier_graph.pb \
  --output_labels=tf_files/trained_models/"${ARCHITECTURE}"/classifier_labels.txt \
  --architecture="${ARCHITECTURE}" \
  --image_dir=tf_files/dataset \
  --random_crop=10 \
  --random_scale=10 \
  --random_brightness=10 \
  --train_batch_size=1000 \
  --validation_batch_size=1000 \
  --eval_step_interval=10
#  --print_misclassified_test_images