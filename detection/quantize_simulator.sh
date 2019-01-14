#!/usr/bin/env bash

~/git/tensorflow/bazel-bin/tensorflow/tools/graph_transforms/transform_graph \
--in_graph=out/sim_model_export/frozen_inference_graph.pb \
--out_graph=out/sim_model_export/frozen_inference_eightbit_graph.pb \
--inputs=image_input \
--outputs="detection_boxes,detection_scores,detection_classes,num_detections" \
--transforms='
add_default_attributes
fold_constants(ignore_errors=true)
fold_batch_norms
fold_old_batch_norms
fuse_resize_and_conv
quantize_weights
strip_unused_nodes
sort_by_execution_order'

#--transforms='
#add_default_attributes
#remove_nodes(op=Identity, op=CheckNumerics)
#fold_constants(ignore_errors=true)
#fold_batch_norms
#fold_old_batch_norms
#fuse_resize_and_conv
#quantize_weights
#quantize_nodes
#strip_unused_nodes
#sort_by_execution_order'