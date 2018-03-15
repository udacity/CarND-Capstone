# Preparing training data

- Annotate images using `labelImg`
- Use `create_voc_tf_record.py` to convert annotation xml files to TFRecord format

# Training

- Train: `python /opt/tensorflow/research/object_detection/train.py --logtostderr --pipeline_config_path ./ssd_mobilenet_sim.config --train_dir ./train_ssd_sim`
- Eval: `python /opt/tensorflow/research/object_detection/eval.py --logtostderr --pipeline_config_path ./ssd_mobilenet_sim.config --eval_dir ./eval_sim`
- Export: python /opt/tensorflow/research/object_detection/export_inference_graph.py --logtostderr --pipeline_config_path ./ssd_mobilenet_sim.config --trained_checkpoint_prefix ./train_ssd_sim/model.ckpt.xxxx --output_dir ./train_ssd_sim/fine_tuned_model`
