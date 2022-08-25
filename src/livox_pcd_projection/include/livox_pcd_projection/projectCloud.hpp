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



class LivoxProjectionNode : public rclcpp::Node
{
public:

    explicit LivoxProjectionNode();

    void getColor(int &result_r, int &result_g, int &result_b, float cur_depth);
    void cloudCallback(const livox_interfaces::msg::CustomMsg& msg);
    string intrinsic_path, extrinsic_path;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr chatter_pub;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr cloud_sub;
    

};

#endif 