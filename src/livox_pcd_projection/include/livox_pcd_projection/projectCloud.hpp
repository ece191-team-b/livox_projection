#ifndef LIVOX_PCD_PROJECTION_PROJECT_CLOUD_CPP
#define LIVOX_PCD_PROJECTION_PROJECT_CLOUD_CPP

#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "livox_interfaces/msg/custom_msg.hpp"
#include <chrono>

#include "common.h"
#include "result_verify.h"
#include <numeric>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <dist_msg/msg/dist.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace cv;
using namespace std::placeholders;


struct projected_pt{
    int u; // image frame x
    int v; // image frame y
    float dist; // distance measurement in meters
};

class LivoxProjectionNode : public rclcpp::Node
{
public:

    explicit LivoxProjectionNode();

    void getColor(int &result_r, int &result_g, int &result_b, float cur_depth);
    void cloudCallback(const livox_interfaces::msg::CustomMsg& msg);
    // void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr& msg);
    void detectionsCallback(const vision_msgs::msg::Detection2DArray& msg);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const livox_interfaces::msg::CustomMsg::ConstSharedPtr &cloud_msg, const vision_msgs::msg::Detection2DArray::ConstSharedPtr &detection_msg) {


    string intrinsic_path, extrinsic_path;
    rclcpp::Publisher<dist_msg::msg::Dist>::SharedPtr chatter_pub;
    message_filters::Subscriber<livox_interfaces::msg::CustomMsg>::SharedPtr cloud_sub;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image>::SharedPtr image_sub;


};

#endif 