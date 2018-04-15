This is the project repo for the Traffic Light Detection of the System Integration Project.

### Generating the inference model frozen_inference_graph.pb

1. This solution uses the Google Object Detection API. Please follow the installation instructions from https://medium.com/@rohitrpatil/how-to-use-tensorflow-object-detection-api-on-windows-102ec8097699

As a result of the installation process you should have [object_detection_path] = \tensorflow\models\research\object_detection and there
* [object_detection_path]\train.py to train the model 
* [object_detection_path]\eval.py to test the model
* [object_detection_path]\export_inference_graph.py to compile the inference model

2. Clone the complete CarND-Capstone respository. The [training_directory] for the Traffic Light Detection is \CarND-Capstone\ros\src\tl_detector\training

3. Download the training and test data provided by Shyam Jagannathan https://drive.google.com/drive/folders/0Bz-TOGv42ojzOHhpaXJFdjdfZTA and copy them to
* [training_directory]\data_sim\traffic-light-sim-train.record
* [training_directory]\data_sim\traffic-light-sim-test.record
* [training_directory]\data_site\traffic-light-site-train.record
* [training_directory]\data_site\traffic-light-site-test.record

4. Download the pretrained model ssd_mobilenet_v1_coco from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md and extract it to [training_directory]\model\ssd_mobilenet\ckpt

5. Now train, test and compile the model:

From [training_directory]  
* train the model:
python [object_detection_path]\train.py --logtostderr --pipeline_config_path=[pipeline.config] --train_dir=[model_path]/train/ 
* test the model: 
python [your object_detection_path]\eval.py --logtostderr --pipeline_config_path=[pipeline.config] --checkpoint_dir=[model_path]/train/ --eval_dir=[model_path]/eval/
* compile the model :
python [your object_detection_path]\export_inference_graph.py --input_type image_tensor --pipeline_config_path=[pipeline.config] --trained_checkpoint_prefix=[model_path]/train/model.ckpt-[number_of_trained_epochs] --output_directory=[model_path]/inference. 

For the simulation the config file [pipeline.config] is pipeline_ssd_v1_sim.config and the directory [model_path] is model_sim. For the site [pipeline.config] is pipeline_ssd_v1_site.config and [model_path] is model_site.

As a result the inference model frozen_inference_graph.pb is generated in [model_path]/inference as input for the Traffic Light Detection tl_detector.py.