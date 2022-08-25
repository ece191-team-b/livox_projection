#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "common.h"
#include "result_verify.h"
// #include "CustomMsg.h"

#include <numeric>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std;
using namespace cv;

void getColor(int &result_r, int &result_g, int &result_b, float cur_depth);

float max_depth = 60;
float min_depth = 3;

cv::Mat src_img;
cv::Mat rectified_img;
cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
cv_bridge::CvImagePtr old_cv_ptr;


vector<livox_pcd_projection::msg::CustomMsg> lidar_datas; 
int threshold_lidar, refresh_rate;  // number of cloud point on the photo
string intrinsic_path, extrinsic_path;

using std::placeholders::_1;

class LivoxProjectionNode(const rclcpp::NodeOptions &options)
: Node("projection_ros2", options),
verbose(true)
{
    this -> declare_parameter<std::string>("intrinsic_path", "");
    this -> declare_parameter<std::string>("extrinsic_path", "");

    intrinsic_path = this -> get_parameter("intrinsic_path").as_string();
    extrinsic_path = this -> get_parameter("extrinsic_path").as_string();

    chatter_pub = n.create_publisher<std_msgs::Float32>("distance", 1000);
    cloud_sub = n.create_subscription<livox_pcd_projection::msg::CustomMsg>("/livox/lidar", 1, std::bind(&LivoxProjectionNode::cloud_callback, this, _1));
    
    image_transport::ImageTransport it(this);
    image_transport::Publisher image_pub = it.advertise("projection_result", 1);
    image_transport::Subscriber sub = it.subscribe("camera/image_0", 1, imageCallback); // subscribe to camera node
    rclcpp::Rate loop_rate(refresh_rate); // loop at 10 Hz
}   


void LivoxProjectionNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // ROS_INFO("Recieved image.");
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

// read pointcloud from msg and save to 
void LivoxProjectionNode::cloudCallback(const livox_interfaces::CustomMsg& msg) {
    // ROS_INFO("Pointcloud recieved.");
    lidar_datas.push_back(msg);
    // set decay rate
    if (lidar_datas.size() > threshold_lidar/24000 + 1) {
        // lidar_datas.clear();
        lidar_datas.erase(lidar_datas.begin());  // pop front
    }
}


void LivoxProjectionNode::run(){
    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    vector<float> distortion;
    getDistortion(intrinsic_path, distortion);
    vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);

	// set intrinsic parameters of the camera
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = intrinsic[0];
    cameraMatrix.at<double>(0, 2) = intrinsic[2];
    cameraMatrix.at<double>(1, 1) = intrinsic[4];
    cameraMatrix.at<double>(1, 2) = intrinsic[5];

	// set radial distortion and tangential distortion
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4];


    // variable initialization
    cv::Mat view, rview, map1, map2;
    vector<float> distances; // stores distances of the lidar point cloud within the bounding box

    // parameters of the mockup bounding box 
    int rect_x = 1488/2;
    int rect_y = 568/2;
    int rect_width = 50;
    int rect_height = 50;
    
    // manually set image size for now
    cv::Size imageSize;
    imageSize.width = 1448;
    imageSize.height = 568;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    
    while (rclcpp::ok()) {
        if (cv_ptr->image.empty() || lidar_datas.empty()) {
           RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No image or pointcloud data received.");
        }
        else {
            src_img = cv_ptr->image;
            if (old_cv_ptr != cv_ptr) {
                

                // read the actual image size
                // cv::Size imageSize = src_img.size();
                // cout << imageSize << endl;

                // int myCount = 0;

                // src_img = cv::imread(input_photo_path); // reread the image every loop
                cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion            
                
                // project the point cloud on to the image
                float x, y, z;
                float theoryUV[2] = {0, 0};
                for (unsigned int i = 0; i < lidar_datas.size(); ++i) {
                    for (unsigned int j = 0; j < lidar_datas[i].point_num; ++j) {
                        x = lidar_datas[i].points[j].x;
                        y = lidar_datas[i].points[j].y;
                        z = lidar_datas[i].points[j].z;

                        getTheoreticalUV(theoryUV, intrinsic, extrinsic, x, y, z);
                        int u = floor(theoryUV[0] + 0.5);
                        int v = floor(theoryUV[1] + 0.5);
                        int r, g, b;
                        getColor(r, g, b, x);

                        // save distance data within the bounding box
                        if (rect_x + rect_width >= u && u >= rect_x && rect_y + rect_width >= v && v >= rect_y) {
                            distances.push_back(x); // TODO: is x the distance to the object? 
                        }
                        Point p(u, v);
                        circle(src_img, p, 1, Scalar(b, g, r), -1);
                        // if (myCount > threshold_lidar) {
                        //     break;
                        // }
                    }
                    // if (myCount > threshold_lidar) {
                    //     break;
                    // }
                }
                
                // draw bounding box
                cv::Rect rect(rect_x, rect_y, rect_width, rect_height);
                cv::rectangle(src_img, rect, cv::Scalar(0, 0, 255)); 

                // publish projected image
                cv_ptr->image = src_img;
                this->image_pub.publish(cv_ptr->toImageMsg());
                
                // display average distance within the bounding box
                float avg = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();

                // create the ROS msg and publish it
                std_msgs::Float32 msg;
                msg.data = avg;
                this->chatter_pub.publish(msg);
                old_cv_ptr = cv_ptr;
                distances.clear();
            }
            // lidar_datas.clear();
        }
        rclcpp::spin_some();
        this.loop_rate.sleep();
    }
}





// set the color by distance to the cloud
void getColor(int &result_r, int &result_g, int &result_b, float cur_depth) {
    float scale = (max_depth - min_depth)/10;
    if (cur_depth < min_depth) {
        result_r = 0;
        result_g = 0;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth + scale) {
        result_r = 0;
        result_g = int((cur_depth - min_depth) / scale * 255) & 0xff;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth + scale*2) {
        result_r = 0;
        result_g = 0xff;
        result_b = (0xff - int((cur_depth - min_depth - scale) / scale * 255)) & 0xff;
    }
    else if (cur_depth < min_depth + scale*4) {
        result_r = int((cur_depth - min_depth - scale*2) / scale * 255) & 0xff;
        result_g = 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth + scale*7) {
        result_r = 0xff;
        result_g = (0xff - int((cur_depth - min_depth - scale*4) / scale * 255)) & 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth + scale*10) {
        result_r = 0xff;
        result_g = 0;
        result_b = int((cur_depth - min_depth - scale*7) / scale * 255) & 0xff;
    }
    else {
        result_r = 0xff;
        result_g = 0;
        result_b = 0xff;
    }

}

// get params from launch file

// read image from msg and save to cv_ptr


/* =========================== MAIN =========================== */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    getParameters();
    
    // setup ROS publisher and subscriber
    auto n = make_shared<rclcpp::Node>("projection_node");
    rclcpp::Publisher<std_msgs::Float32> chatter_pub = n.create_publisher<std_msgs::Float32>("distance", 1000);
    rclcpp::Subscriber cloud_sub = n.create_subscription<livox_pcd_projection::msg::CustomMsg>("/livox/lidar", 1, cloudCallback);
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub = it.advertise("projection_result", 1);
    image_transport::Subscriber sub = it.subscribe("camera/image_0", 1, imageCallback); // subscribe to camera node
    rclcpp::Rate loop_rate(refresh_rate); // loop at 10 Hz

    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    vector<float> distortion;
    getDistortion(intrinsic_path, distortion);
    vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);

	// set intrinsic parameters of the camera
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = intrinsic[0];
    cameraMatrix.at<double>(0, 2) = intrinsic[2];
    cameraMatrix.at<double>(1, 1) = intrinsic[4];
    cameraMatrix.at<double>(1, 2) = intrinsic[5];

	// set radial distortion and tangential distortion
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4];


    // variable initialization
    cv::Mat view, rview, map1, map2;
    vector<float> distances; // stores distances of the lidar point cloud within the bounding box

    // parameters of the mockup bounding box 
    int rect_x = 1488/2;
    int rect_y = 568/2;
    int rect_width = 50;
    int rect_height = 50;
    
    // manually set image size for now
    cv::Size imageSize;
    imageSize.width = 1448;
    imageSize.height = 568;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    
    while (rclcpp::ok()) {
        if (cv_ptr->image.empty() || lidar_datas.empty()) {
           RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No image or pointcloud data received.");
        }
        else {
            src_img = cv_ptr->image;
            if (old_cv_ptr != cv_ptr) {
                

                // read the actual image size
                // cv::Size imageSize = src_img.size();
                // cout << imageSize << endl;

                // int myCount = 0;

                // src_img = cv::imread(input_photo_path); // reread the image every loop
                cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion            
                
                // project the point cloud on to the image
                float x, y, z;
                float theoryUV[2] = {0, 0};
                for (unsigned int i = 0; i < lidar_datas.size(); ++i) {
                    for (unsigned int j = 0; j < lidar_datas[i].point_num; ++j) {
                        x = lidar_datas[i].points[j].x;
                        y = lidar_datas[i].points[j].y;
                        z = lidar_datas[i].points[j].z;

                        getTheoreticalUV(theoryUV, intrinsic, extrinsic, x, y, z);
                        int u = floor(theoryUV[0] + 0.5);
                        int v = floor(theoryUV[1] + 0.5);
                        int r, g, b;
                        getColor(r, g, b, x);

                        // save distance data within the bounding box
                        if (rect_x + rect_width >= u && u >= rect_x && rect_y + rect_width >= v && v >= rect_y) {
                            distances.push_back(x); // TODO: is x the distance to the object? 
                        }
                        Point p(u, v);
                        circle(src_img, p, 1, Scalar(b, g, r), -1);
                        // if (myCount > threshold_lidar) {
                        //     break;
                        // }
                    }
                    // if (myCount > threshold_lidar) {
                    //     break;
                    // }
                }
                
                // draw bounding box
                cv::Rect rect(rect_x, rect_y, rect_width, rect_height);
                cv::rectangle(src_img, rect, cv::Scalar(0, 0, 255)); 

                // publish projected image
                cv_ptr->image = src_img;
                image_pub.publish(cv_ptr->toImageMsg());
                
                // display average distance within the bounding box
                float avg = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();

                // create the ROS msg and publish it
                std_msgs::Float32 msg;
                msg.data = avg;
                ROS_INFO("Publishing to topic /distance: %f", avg);
                chatter_pub.publish(msg);
                old_cv_ptr = cv_ptr;
                distances.clear();
            }
            // lidar_datas.clear();
        }
        rclcpp::spin_some();
        loop_rate.sleep();
    }

    cv::imwrite(output_path, src_img);
    return 0;
}



