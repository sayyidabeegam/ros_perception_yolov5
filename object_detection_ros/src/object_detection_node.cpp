#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <string>
#include <fstream>  
#include "detector.h"

YOLODetector detector(nullptr);
image_transport::Publisher image_pub;
std::vector<std::string> classNames;
std::string image_topic;

void loadClassNames(const std::string& classNamesPath) {
    std::ifstream classNamesFile(classNamesPath);
    if (classNamesFile.is_open()) {
        std::string className;
        while (std::getline(classNamesFile, className)) {
            classNames.push_back(className);
        }
        classNamesFile.close();
    } else {
        ROS_ERROR("Failed to open class names file: %s", classNamesPath.c_str());
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;

        // Perform object detection
        const float confThreshold = 0.3f;
        const float iouThreshold = 0.4f;
        std::vector<Detection> result = detector.detect(image, confThreshold, iouThreshold);

        // Annotate image with bounding boxes and class names
        for (const auto& detection : result) {
            cv::Rect box = detection.box;
            std::string className = detection.classId < classNames.size() ? classNames[detection.classId] : "Unknown";
            // Draw bounding box
            cv::rectangle(image, box, cv::Scalar(0, 255, 0), 2);

            // Put class name text
            cv::Point textOrg(box.x, box.y - 10);  // Position above the bounding box
            cv::putText(image, className, textOrg, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            ROS_INFO("Detected class: %s", className.c_str());
        }

        // Convert annotated OpenCV image back to ROS image message
        cv_ptr->image = image;
        sensor_msgs::ImagePtr annotated_img_msg = cv_ptr->toImageMsg();

        // Publish annotated image
        image_pub.publish(annotated_img_msg);
    } catch (const std::exception& e) {
        ROS_ERROR("Error processing image: %s", e.what());
    }
}

int main(int argc, char* argv[]) {
    // Initialize ROS node
    ros::init(argc, argv, "object_detection_node");
    ros::NodeHandle nh;

    // Check if an image topic argument is provided
    if (argc < 2) {
        ROS_ERROR("Usage: rosrun object_detection_ros object_detection_node <image_topic>");
        return 1;
    }

    // Set the image topic from command-line argument
    image_topic = argv[1];
    std::string modelPath, classNamesPath;
    nh.param<std::string>("/object_detection_node/model_path", modelPath, "config/yolov5n.onnx");
    nh.param<std::string>("/object_detection_node/class_names_path", classNamesPath, "config/classes.txt");

    // Define ROS image transport for subscribing to specified camera topic
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(image_topic, 1, imageCallback);

    // Define image_transport::Publisher for annotated image with bounding boxes
    image_pub = it.advertise("/perception_output", 1);

    // Load YOLO model and initialize detector
    modelPath = ros::package::getPath("object_detection_ros") + "/" + modelPath;
    bool isGPU = false; 

    try {
        detector = YOLODetector(modelPath, isGPU, cv::Size(640, 640));
        ROS_INFO("Model was initialized.");
    } catch(const std::exception& e) {
        ROS_ERROR("Error initializing model: %s", e.what());
        return -1;
    }

    // Load class names
    classNamesPath = ros::package::getPath("object_detection_ros") + "/" + classNamesPath;
    loadClassNames(classNamesPath);

    // ROS spin to process callbacks
    ros::spin();

    return 0;
}
