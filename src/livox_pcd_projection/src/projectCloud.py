import rclpy
import cv2 
from rclpy.node import Node
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
    ObjectHypothesis
)
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image
from livox_interfaces.msg import CustomMsg
import numpy as np

class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')
        self.declare_parameter('camera_topic', '/livox_camera/image_raw')
        self.declare_parameter('detection_topic', '/livox_camera/detection')
        self.declare_parameter('lidar_topic', '/livox_camera/projection')
        self.declare_parameter('camera_info_topic', '/livox_camera/camera_info')
        self.declare_parameter('intrinsic_path', '/home/robot/catkin_ws/src/livox_pcd_projection/config/intrinsic.yml')
        self.declare_parameter('extrinsic_path', '/home/robot/catkin_ws/src/livox_pcd_projection/config/extrinsic.yml')
        self.declare_parameter('lidar_threshold', 20000)
        self.declare_parameter('refresh_rate', 10)
        self.declare_parameter('debug', False)


        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.detection_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.lidar_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.intrinsic_path = self.get_parameter('intrinsic_path').get_parameter_value().string_value
        self.extrinsic_path = self.get_parameter('extrinsic_path').get_parameter_value().string_value
        self.lidar_threshold = self.get_parameter('lidar_threshold').get_parameter_value().integer_value
        self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.lidar_decay_list = []

        if self.debug:
            self.camera_sub = message_filters.Subscriber(self, Image, self.camera_topic)
            self.detection_sub = message_filters.Subscriber(self, Detection2DArray, self.detection_topic)
            self.lidar_sub = message_filters.Subscriber(self, CustomMsg, self.lidar_topic)
            ts = message_filters.ApproximateTimeSynchronizer([self.camera_sub, self.detection_sub, self.lidar_sub], 10, 0.1)
            ts.registerCallback(self.debug_callback)

        else :
            self.camera_sub = message_filters.Subscriber(self, Image, self.camera_topic)
            self.detection_sub = message_filters.Subscriber(self, Detection2DArray, self.detection_topic)
            ts = message_filters.ApproximateTimeSynchronizer([self.camera_sub, self.detection_sub], 10, 0.1)
            ts.registerCallback(self.callback)

    def debug_callback(self, camera_msg, detection_msg, lidar_msg):
        self.get_logger().info('Received message')
        projected_points = []
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(camera_msg, desired_encoding='passthrough')
        if len(self.lidar_decay_list) > self.lidar_threshold / 24000 + 1:
           self.lidar_decay_list.pop(0)
        self.lidar_decay_list.append(lidar_msg)
        for i in range(len(self.lidar_decay_list)):
            for j in range(len(self.lidar_decay_list[i].point_num)):
                x =  self.lidar_decay_list[i].point[j].x
                y =  self.lidar_decay_list[i].point[j].y
                z =  self.lidar_decay_list[i].point[j].z
                u, v = self.getTheoreticalUV(self.intrinsic, self.extrinsic, x, y, z)
                projected_points.append([x, u, v])
                
        
    def load_extrinsic(self, extrinsic_path):
        file = open(extrinsic_path, 'r')
        lines = file.readlines()
        extrinsic = np.zeros((4, 4))
        for i, line in enumerate(lines):
            if i == 0:
                continue
            else:
                line = line.split('  ')
                extrinsic[i-1]= [float(x) for x in line]
    
        file.close()
        self.extrinsic = extrinsic

    def load_intrinsic_distortion(self, intrinsic_path):
        file = open(intrinsic_path, 'r')
        lines = file.readlines()
        intrinsic = np.zeros((3, 3))
        distortion = np.zeros((5, 1))
        skip = [0, 4, 5]
        for i, line in enumerate(lines):
            if i in skip:
                continue
            elif i < 3:
                line = line.split(' ')
                print(line)
                intrinsic[i-1]= [float(x) for x in line if x != '']
            else:
                line = line.split(' ')
                for j, dist in enumerate(line):
                    if dist != '':
                        distortion[j] = float(dist)
        self.intrinsic = intrinsic
        self.distortion = distortion
        file.close()

    def getTheoreticalUV(intrinsic, extrinsic, x, y, z):
        # intrinsic is 3x3
        # extrinsic is 3x4
        # m3 4x1
        m3 = np.array([x, y, z, 1])
        result = intrinsic @ extrinsic[:3, :] @ m3.T

        depth = result[2]
        u = result[0] / depth
        v = result[1] / depth

        return u, v, depth

