#include "livox_pcd_projection/projectCloud.hpp"

using namespace std;
using namespace cv;

float max_depth = 60;
float min_depth = 3;

cv::Mat src_img;
cv::Mat rectified_img;
cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
cv_bridge::CvImagePtr old_cv_ptr;

// function declaration
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

vector<livox_interfaces::msg::CustomMsg> lidar_datas; 
vector<projected_pt> projected_pts;
vision_msgs::msg::Detection2DArray detection_msg;
string intrinsic_path, extrinsic_path, camera_topic, lidar_topic, detection_topic;
int threshold_lidar, refresh_rate;  // number of pointcloud points projected onto image
bool debug; // switch for debug mode

using std::placeholders::_1;

LivoxProjectionNode::LivoxProjectionNode(): Node("Projection") {
    // declare parameters and their default value
    this->declare_parameter<std::string>("intrinsic_path", "");
    this->declare_parameter<std::string>("extrinsic_path", "");
    this->declare_parameter<std::string>("camera_topic", "");
    this->declare_parameter<std::string>("lidar_topic", "");
    this->declare_parameter<std::string>("detection_topic", "");
    this->declare_parameter<int>("lidar_threshold", 20000);
    this->declare_parameter<int>("refresh_rate", 10);
    this->declare_parameter<bool>("debug", false);

    // get parameters from launch file
    intrinsic_path = this->get_parameter("intrinsic_path").as_string();
    extrinsic_path = this->get_parameter("extrinsic_path").as_string();
    camera_topic = this->get_parameter("camera_topic").as_string();            
    lidar_topic = this->get_parameter("lidar_topic").as_string();
    detection_topic = this->get_parameter("detection_topic").as_string();
    threshold_lidar = this->get_parameter("lidar_threshold").as_int();
    refresh_rate = this->get_parameter("refresh_rate").as_int();
    debug = this->get_parameter("debug").as_bool();

    RCLCPP_INFO(this->get_logger(), "Extrinsic path: %s", intrinsic_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Intrinsic path: %s", extrinsic_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Lidar topic: %s", lidar_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Detection topic: %s", detection_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Lidar threshold: %d", threshold_lidar);
    RCLCPP_INFO(this->get_logger(), "Refresh rate is set to: %d", refresh_rate);
    RCLCPP_INFO(this->get_logger(), "Debug mode: %s", debug ? "true":"false");

    // initialize publisher and subscriber
    chatter_pub = this->create_publisher<dist_msg::msg::Dist>("distances", 10);
    cloud_sub = this->create_subscription<livox_interfaces::msg::CustomMsg>(lidar_topic, 1, std::bind(&LivoxProjectionNode::cloudCallback, this, _1));
    detection_sub = this->create_subscription<vision_msgs::msg::Detection2DArray>(detection_topic, 1, std::bind(&LivoxProjectionNode::detectionsCallback, this, _1));
    
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(this.get(), camera_topic, 1);
    message_filters::Subscriber<livox_interfaces::msg::CustomMsg> cloud_sub(this.get(), lidar_topic, 1);
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detection_sub(this.get(), detection_topic, 1);
    typedef sync_policies::ApproximateTime<sensor_msg::msg::Image, livox_interfaces::msg::CustomMsg, vision_msgs::msg::Detection2DArray> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cloud_sub, detection_sub);
    sync.registerCallback(std::bind(&LivoxProjectionNode::callback, _1, _2, _3));

}


void LivoxProjectionNode::callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const livox_interfaces::msg::CustomMsg::ConstSharedPtr &cloud_msg, const vision_msgs::msg::Detection2DArray::ConstSharedPtr &detection_msg) {
    // do something
    RCLCPP_INFO(this->get_logger(), "callback");
}


// set the color by distance to the cloud
void LivoxProjectionNode::getColor(int &result_r, int &result_g, int &result_b, float cur_depth) {
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

// copy the image ptr from subscribed image msg
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
    try {
        // ROS_INFO("Recieved image.");
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not convert from to 'bgr8'.");
    }
}

// read pointcloud from msg and save to 
void LivoxProjectionNode::cloudCallback(const livox_interfaces::msg::CustomMsg& msg) {
    // ROS_INFO("Pointcloud recieved.");
    lidar_datas.push_back(msg);
    // set decay rate
    if (lidar_datas.size() > threshold_lidar/24000 + 1) {
        // lidar_datas.clear();
        lidar_datas.erase(lidar_datas.begin());  // pop front
    }
}

// detection msg callback
void LivoxProjectionNode::detectionsCallback(const vision_msgs::msg::Detection2DArray& msg) {
    RCLCPP_INFO(this->get_logger(), "Recived detection message with stamp:  %i", msg.header.stamp.sec);
    detection_msg = msg;
}


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto p = std::make_shared<LivoxProjectionNode>();
    RCLCPP_INFO(p->get_logger(), "Initializing LivoxProjectionNode");
    rclcpp.spin(p);

    // vector<float> intrinsic;
    // getIntrinsic(p->intrinsic_path, intrinsic);
    // vector<float> distortion;
    // getDistortion(p->intrinsic_path, distortion);
    // vector<float> extrinsic;
    // getExtrinsic(p->extrinsic_path, extrinsic);

	// // set intrinsic parameters of the camera
    // RCLCPP_INFO(p->get_logger(), "Setting the instrinsic parameters of the camera");
    // cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    // cameraMatrix.at<double>(0, 0) = intrinsic[0];
    // cameraMatrix.at<double>(0, 2) = intrinsic[2];
    // cameraMatrix.at<double>(1, 1) = intrinsic[4];
    // cameraMatrix.at<double>(1, 2) = intrinsic[5];

	// // set radial distortion and tangential distortion
    // cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    // distCoeffs.at<double>(0, 0) = distortion[0];
    // distCoeffs.at<double>(1, 0) = distortion[1];
    // distCoeffs.at<double>(2, 0) = distortion[2];
    // distCoeffs.at<double>(3, 0) = distortion[3];
    // distCoeffs.at<double>(4, 0) = distortion[4];

    // // variable initialization
    // cv::Mat view, rview, map1, map2;
    // vector<float> distances; // stores distances of the lidar point cloud within the bounding box
    
    // // if (debug) {
    // //     rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher");
    // //     image_transport::ImageTransport it(node);
    // //     image_transport::Publisher image_pub = it.advertise("projection_result", 1);
    // //     image_transport::Subscriber sub = it.subscribe(camera_topic, 1, imageCallback); // subscribe to camera node
    // // }

    // rclcpp::Rate loop_rate(refresh_rate);

    // int timeout_counter = 0;
    // int timeout = 5; // unit: seconds
    // dist_msg::msg::Dist distances_msg;
    
    // // if (debug) {
    // //     cv::Size imageSize; 
    // //     imageSize.width = 1448;
    // //     imageSize.height = 568;
    // //     cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    // // }
    // // TODO: make logging when no data is coming nicer, change to try maybe

    // while (rclcpp::ok()) {
    //     // if (!cv_ptr->image.empty() && !lidar_datas.empty()) {
    //     if (!lidar_datas.empty()) {
    //         // cv::Size imageSize = cv_ptr->image.size();
    //         // RCLCPP_INFO_ONCE(p->get_logger(), "Rectifying camera distortion");
    //         // cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    //         RCLCPP_INFO_ONCE(p->get_logger(), "Start pointcloud projection!");
    //         timeout_counter = 0; // reset timeout counter

    //         // src_img = cv_ptr->image;
    //         // if (old_cv_ptr != cv_ptr) {
            
    //             // cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion            
                
    //             // project the point cloud on to the image
    //         float x, y, z;
    //         float theoryUV[2] = {0, 0};
    //         for (unsigned int i = 0; i < lidar_datas.size(); ++i) {
    //             for (unsigned int j = 0; j < lidar_datas[i].point_num; ++j) {
    //                 x = lidar_datas[i].points[j].x;
    //                 y = lidar_datas[i].points[j].y;
    //                 z = lidar_datas[i].points[j].z;

    //                 getTheoreticalUV(theoryUV, intrinsic, extrinsic, x, y, z);
    //                 int u = floor(theoryUV[0] + 0.5);
    //                 int v = floor(theoryUV[1] + 0.5);

    //                 projected_pts.push_back({u, v, x}); // store projected points in list

    //                 // if (debug) {
    //                 //     int r, g, b;
    //                 //     p->getColor(r, g, b, x);
    //                 //     Point pt(u, v);
    //                 //     circle(src_img, pt, 1, Scalar(b, g, r), -1);
    //                 // }
    //             }
    //         }

    //         float avg; 
    //         // iterate over all the bounding boxes
    //         while (!detection_msg.detections.empty()) {
    //             // extract info from msg
    //             vision_msgs::msg::Detection2D detection = detection_msg.detections[0];
    //             vision_msgs::msg::ObjectHypothesisWithPose results = detection.results[0];
                
    //             // extract bounding box info
    //             vision_msgs::msg::BoundingBox2D bbox2d = detection.bbox;
    //             geometry_msgs::msg::Pose2D center = bbox2d.center;

    //             RCLCPP_INFO(p->get_logger(), "detection object class id: %s with score %f", results.hypothesis.class_id.c_str(), results.hypothesis.score);
    //             RCLCPP_INFO(p->get_logger(), "   position (%f, %f)", center.x, center.y);
    //             RCLCPP_INFO(p->get_logger(), "   size %f x %f", bbox2d.size_x, bbox2d.size_y);

    //             for (projected_pt pt: projected_pts) {
    //                 if ((center.x + bbox2d.size_x / 2) >= pt.u && (center.x - bbox2d.size_x / 2) <= pt.u && (center.y + bbox2d.size_y / 2) >= pt.v && (center.y - bbox2d.size_y / 2) <= pt.u) {
    //                     if (pt.dist != nan("")) {
    //                         distances.push_back(pt.dist);
    //                     }
    //                 }
    //             }

    //             avg = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size(); // calculate average distance for this boundingbox
    //             distances_msg.distances.push_back(avg);

    //             RCLCPP_INFO(p->get_logger(), "Average distance within in bounding box is %f", avg);
    //             // cv::Rect rect(center.x - bbox2d.size_x / 2, center.y - bbox2d.size_y / 2, bbox2d.size_x, bbox2d.size_y);
    //             // cv::rectangle(src_img, rect, cv::Scalar(0, 0, 255), 5); 
    //             distances.clear();
    //             detection_msg.detections.erase(detection_msg.detections.begin()); // pop front
    //         }

    //         // save distance data within the bounding box
    //         // if (rect_x + rect_width >= u && u >= rect_x && rect_y + rect_width >= v && v >= rect_y) {
    //         //     distances.push_back(x); // TODO: is x the distance to the object? 
    //         // }
            
    //         // draw bounding box
    //         // cv::Rect rect(rect_x, rect_y, rect_width, rect_height);
    //         // cv::rectangle(src_img, rect, cv::Scalar(0, 0, 255)); 

    //         // publish projected image
    //         // if (debug) {
    //             // cv_ptr->image = src_img;
    //             // image_pub.publish(cv_ptr->toImageMsg());
    //         // }
            
    //         // display average distance within the bounding box
    //         // float avg = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    //         // RCLCPP_INFO(p->get_logger(), "Measured distance: %f", avg);

    //         p->chatter_pub->publish(distances_msg);
    //         // old_cv_ptr = cv_ptr;
    //         distances_msg.distances.clear();
    //     // }
    //     }
    //     else {
    //         RCLCPP_INFO_ONCE(p->get_logger(), "Waiting for lidar pointcloud data...");
    //         RCLCPP_INFO(p->get_logger(), "timeout counter: %d", timeout_counter);
    //         timeout_counter++;
    //         if (timeout_counter > timeout * refresh_rate) {
    //             RCLCPP_ERROR(p->get_logger(), "Missing image and/or lidar data!");
    //             return 1;
    //         }
    //     }
    //     // rclcpp::spin_some(node);
    //     rclcpp::spin_some(p);
    //     loop_rate.sleep();
    // }
}


// // get params from launch file

// // read image from msg and save to cv_ptr


// /* =========================== MAIN =========================== */
// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<LivoxProjection>());
//     rclcpp::shutdown();
// }


// int main(){
//     cout << "Hello World!" << endl;
// }

// rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher");

// image_transport::ImageTransport it(node);
// image_transport::Publisher image_pub = it.advertise("projection_result", 1);
// image_transport::Subscriber sub = it.subscribe("camera/image_0", 1, std::bind(&LivoxProjectionNode::imageCallback, this, _1)); // subscribe to camera node