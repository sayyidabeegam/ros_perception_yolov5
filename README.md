# ros_perception_yolov5 
The ROS node, named object_detection_node, is designed to perform real-time object detection on images received from a specified ROS image topic. The implementation uses a YOLOV5 model for detecting objects within the images. Detected objects are annotated with bounding boxes and class names before being republished on another ROS image topic (/perception_output).

## Requirments
1. ROS Noetic
2. OpencV
3. ONNX

## Usage
### Docker
1. Pull the docker image
```
docker pull sayyidabeegam/ros-perception:v1
```
2. Run docker container and run roscore on the same terminal
```
docker run -it --name <name of container> sayyidabeegam/ros-perception:v1
source /opt/ros/noetic/setup.bash
roscore
```
3. Check container ID
```
docker ps
```
4. Run rosbag on another terminal
```
docker exec -it <container ID> bash
source /opt/ros/noetic/setup.bash
rosbag play <your bag name>
```
5. Launch the ros package in another terminal
```
docker exec -it <container ID> bash
cd catkin_ws
source devel/setup.bash
roslaunch object_detection_ros object_detection.launch image_topic:=<input image topic>
```
### Native
Clone repo and build it
```
git clone <This repo to your ws/src>
cd ..
catkin_make
```
Test the package
Terminal 1: roscore
Terminal 2: Play your bag file to test
Terminal 3:
```
cd <your ws>
source devel/setup.bash
roslaunch object_detection_ros object_detection.launch image_topic:=<input image topic>
```

## Output


https://github.com/sayyidabeegam/ros_perception_yolov5/assets/47295006/4a240859-7594-415c-83ba-fb8622020672



## Reference
1. [yolov5-onnxruntime ](https://github.com/itsnine/yolov5-onnxruntime)
2. [Setting up ONNX Runtime on Ubuntu 20.04 (C++ API)](https://stackoverflow.com/questions/63420533/setting-up-onnx-runtime-on-ubuntu-20-04-c-api)
