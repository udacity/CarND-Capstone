IMAGE_SIZE=224
ARCHITECTURE="mobilenet_1.0_${IMAGE_SIZE}"

python retrain.py \
  --bottleneck_dir=tf_files/bottlenecks \
  --how_many_training_steps=4000 \
  --model_dir=tf_files/models/ \
  --summaries_dir=tf_files/training_summaries/"${ARCHITECTURE}" \
  --output_graph=tf_files/classifier_graph.pb \
  --output_labels=tf_files/classifier_labels.txt \
  --architecture="${ARCHITECTURE}" \
  --image_dir=tf_files/dataset \
  --random_crop=10 \
  --random_scale=10 \
  --random_brightness=10 \
  --validation_batch_size=-1 \
  --print_misclassified_test_images