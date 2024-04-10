# perception_yolov5_ros
This repository contains a rospackage that perform real time object detection on camera images using YOLOV5. The node subscribe the input image topic and detect the object and publish the detection output an another ROS topic.

## Requirments
1. ROS Noetic
2. OpencV
3. ONNX

## Usage
```
git clone <This repo to your ws/src>
cd ..
catkin_make
source devel/setup.bash
roslaunch object_detection_ros object_detection.launch image_topic:=<input image topic>
```
