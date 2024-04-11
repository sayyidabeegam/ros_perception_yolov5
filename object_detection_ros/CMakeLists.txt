cmake_minimum_required(VERSION 3.0.2)
project(object_detection_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  geometry_msgs  
)
find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS xphoto calib3d)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Find ONNX Runtime
set(ONNXRUNTIME_DIR /tmp/onnxInstall/build/native)  # Path to ONNX Runtime installation
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS ${ONNXRUNTIME_DIR}/include)
find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    HINTS ${ONNXRUNTIME_DIR}/lib)

# Check if ONNX Runtime was found
if (ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_INCLUDE_DIR}, ${ONNXRUNTIME_LIBRARY}")
    include_directories(${ONNXRUNTIME_INCLUDE_DIR})
else ()
    message(FATAL_ERROR "ONNX Runtime not found. Set ONNXRUNTIME_DIR to the root directory of your ONNX Runtime installation.")
endif ()

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${ONNXRUNTIME_INCLUDE_DIR}
)

add_executable(object_detection_node src/object_detection_node.cpp
                src/detector.cpp
                src/utils.cpp)
target_link_libraries(object_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ONNXRUNTIME_LIBRARY})
