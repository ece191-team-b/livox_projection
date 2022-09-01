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
import torch

class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')
        self.declare_parameter('camera_topic', '/right_camera/image')
        self.declare_parameter('detection_topic', '/right_set/bbox')
        self.declare_parameter('lidar_topic', '/livox/lidar_3WEDH7600108721')
        # self.declare_parameter('camera_info_topic', '/livox_camera/camera_info')
        self.declare_parameter('intrinsic_path', '/home/inspirationagx01/livox_projection/calibration_data/parameters/intrinsic.txt')
        self.declare_parameter('extrinsic_path', '/home/inspirationagx01/livox_projection/calibration_data/parameters/extrinsic.txt')
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
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.get_logger().info("Finished parsing parameters")

        self.load_extrinsic(self.extrinsic_path)
        self.load_intrinsic_distortion(self.intrinsic_path)

        self.lidar_decay_list = []

        if self.debug:
            self.camera_sub = message_filters.Subscriber(self, Image, self.camera_topic)
            self.detection_sub = message_filters.Subscriber(self, Detection2DArray, self.detection_topic)
            self.lidar_sub = message_filters.Subscriber(self, CustomMsg, self.lidar_topic)
            ts = message_filters.ApproximateTimeSynchronizer([self.camera_sub, self.detection_sub, self.lidar_sub], 10, 0.1)
            ts.registerCallback(self.debug_callback)

        else :
            self.get_logger().info("Starting Subscribers and callback")
            self.lidar_sub = message_filters.Subscriber(self, CustomMsg, self.lidar_topic)
            # self.lidar_sub = message_filters.Subscriber(self, self.lidar_topic, CustomMsg)

            self.detection_sub = message_filters.Subscriber(self, Detection2DArray, self.detection_topic)
            # self.detection_sub = message_filters.Subscriber(self, self.detection_topic, Detection2DArray)

            ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.detection_sub], 10, 0.1, allow_headerless=True)
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
                
    def callback(self, detection_msg, lidar_msg):
        self.get_logger().info('Recieved message')
    
        
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
        self.extrinsic = torch.from_numpy(extrinsic).to(self.device)
        self.get_logger().info('Loaded extrinsic data')

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
                # print(line)
                intrinsic[i-1]= [float(x) for x in line if x != '']
            else:
                line = line.split(' ')
                for j, dist in enumerate(line):
                    if dist != '':
                        distortion[j] = float(dist)
        self.intrinsic = torch.from_numpy(intrinsic).to(self.device)
        self.distortion = torch.from_numpy(distortion).to(self.device)
        file.close()
        self.get_logger().info('Loaded intrinsic data')

    def getTheoreticalUV(self, x, y, z):
        # intrinsic is 3x3
        # extrinsic is 3x4
        # m3 4x1
        m3 = torch.tensor([x, y, z, 1]).to(self.device)

        result = torch.mm(self.intrinsic @ self.extrinsic[:3, :] @ m3.T)

        result = result.cpu()

        depth = result[2]
        u = result[0] / depth
        v = result[1] / depth

        return u, v, depth


def main(args=None):
    rclpy.init(args=args)

    node = ProjectionNode()

    rclpy.spin(node)

if __name__ == '__main__':
    main()