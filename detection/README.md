# Tools and data to train a detection model


## Requirements

Libraries are a little out of date as the target platform is using
tensorflow 1.3.0.

- python 2.7
- protoc2.6
- a nvidia gpu that works with tensorflow (1.3.0)
- [CUDA 8.0](https://developer.nvidia.com/cuda-80-ga2-download-archive)
- [CuDNN 6.0](https://developer.nvidia.com/rdp/cudnn-archive)

Test tensorflow installation:

```python
#> python
import tensorflow as tf
tf.InteractiveSession();

```

Should give no errors and show your gpu info. Like:

```
2019-01-13 12:03:49.647675: I tensorflow/core/common_runtime/gpu/gpu_device.cc:955] Found device 0 with properties: 
name: GeForce GTX 1070
major: 6 minor: 1 memoryClockRate (GHz) 1.7085
pciBusID 0000:3e:00.0
Total memory: 7.92GiB
Free memory: 7.83GiB

```

On some machines, you need to make sure you have the right kernel modules are loaded:

```sh
#> sudo prime-select nvidia
```

## Installation

- ./install.sh
- source setup.bash
- ./test-env.sh

## Usage

- source setup.bash
- ./prepare_data.sh
- ./train_simulator.sh
- ./export_graph_simulator.sh
- ./test_simulator.sh

## References:
- [Tensorflow Object Detection](https://github.com/tensorflow/models/tree/0375c800c767db2ef070cee1529d8a50f42d1042/object_detection)
- [Traffic Light Detection Using the TensorFlow* Object Detection API](https://software.intel.com/en-us/articles/traffic-light-detection-using-the-tensorflow-object-detection-api)
