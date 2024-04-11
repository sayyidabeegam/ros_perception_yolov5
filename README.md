# ros_perception_yolov5 
This repository contains a rospackage that perform real time object detection on camera images using YOLOV5. The node subscribe the input image topic and detect the object and publish the detection output an another ROS topic.

## Requirments
1. ROS Noetic
2. OpencV
3. ONNX

## Usage
### Docker
```
docker pull sayyidabeegam/ros-perception:v1
```
### Build in native
```
git clone <This repo to your ws/src>
cd ..
catkin_make
source devel/setup.bash
roslaunch object_detection_ros object_detection.launch image_topic:=<input image topic>
```
## Output


https://github.com/sayyidabeegam/ros_perception_yolov5/assets/47295006/4a240859-7594-415c-83ba-fb8622020672



## Reference
1. [yolov5-onnxruntime ](https://github.com/itsnine/yolov5-onnxruntime)
2. [Setting up ONNX Runtime on Ubuntu 20.04 (C++ API)](https://stackoverflow.com/questions/63420533/setting-up-onnx-runtime-on-ubuntu-20-04-c-api)
