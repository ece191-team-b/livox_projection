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
from sensor_msgs.msg import Image, CameraInfo
from livox_interfaces.msg import CustomMsg
import numpy as np
import cupy as cp
import time

class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')
        self.declare_parameter('camera_topic', '/right_camera/image')
        self.declare_parameter('detection_topic', '/right_set/bbox')
        self.declare_parameter('lidar_topic', '/livox/stamped')
        self.declare_parameter('intrinsic_path', '/home/inspirationagx01/livox_projection/calibration_data/parameters/intrinsic.txt')
        self.declare_parameter('extrinsic_path', '/home/inspirationagx01/livox_projection/calibration_data/parameters/extrinsic.txt')
        self.declare_parameter('lidar_threshold', 20000)
        self.declare_parameter('refresh_rate', 30)
        self.declare_parameter('debug', False)


        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.intrinsic_path = self.get_parameter('intrinsic_path').get_parameter_value().string_value
        self.extrinsic_path = self.get_parameter('extrinsic_path').get_parameter_value().string_value
        self.lidar_threshold = self.get_parameter('lidar_threshold').get_parameter_value().integer_value
        self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        
        
        
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"Detection topic: {self.detection_topic}")
        self.get_logger().info(f"Lidar topic: {self.lidar_topic}")

        self.get_logger().info("Finished parsing parameters")

        self.load_extrinsic(self.extrinsic_path)
        self.load_intrinsic_distortion(self.intrinsic_path)

        self.lidar_decay_list = []

        self.get_logger().info("Starting Subscribers and callback")
        
        # create subscriber to bbox and pointcloud
        self.detection_sub = message_filters.Subscriber(self, Detection2DArray, self.detection_topic)
        self.lidar_sub = message_filters.Subscriber(self, CustomMsg, self.lidar_topic)

        if self.debug:
            # also subscribe to image for visualization purposes
            self.camera_sub = message_filters.Subscriber(self, Image, self.camera_topic)
            self.ts = message_filters.ApproximateTimeSynchronizer([self.camera_sub, self.detection_sub, self.lidar_sub], 10, 0.1)
            self.ts.registerCallback(self.debug_callback)
        else :
            self.ts = message_filters.ApproximateTimeSynchronizer([self.detection_sub, self.lidar_sub], 10, 1)
            self.ts.registerCallback(self.callback)

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
        projected_points = cp.array([])
        if len(self.lidar_decay_list) > self.lidar_threshold / 24000 + 1:
           self.lidar_decay_list.pop(0)
        self.lidar_decay_list.append(lidar_msg)

        start_t = time.time()
        # for i in range(len(self.lidar_decay_list)):
        #     for j in range(self.lidar_decay_list[i].point_num):
        #         x =  self.lidar_decay_list[i].points[j].x
        #         y =  self.lidar_decay_list[i].points[j].y
        #         z =  self.lidar_decay_list[i].points[j].z
        #         u, v = self.getTheoreticalUV(x, y, z, projected_points)
        #         projected_points.append([x, u, v])
        x, y, z, = [], [], []

        for i in range(len(self.lidar_decay_list)):
            x = [self.lidar_decay_list[i].point[j].x for j in range(self.lidar_decay_list[i].point_num)]
            y = [self.lidar_decay_list[i].point[j].y for j in range(self.lidar_decay_list[i].point_num)]
            z = [self.lidar_decay_list[i].point[j].z for j in range(self.lidar_decay_list[i].point_num)]

            x, y, z = np.array(x)[:,None], np.array(y)[:,None], np.array(z)[:,None]
            print(x.shape)

                # u, v = self.getTheoreticalUV(x, y, z, projected_points)
                # projected_points.append([x, u, v])


        print(time.time() - start_t)
        # print("done get UVV")

        self.get_logger().info(str(len(detection_msg.detections)))
        for detection in detection_msg.detections:
            bbox = detection.bbox
            pose2d = bbox.center
            center_x, center_y = pose2d
            width, length = bbox
            self.get_logger().info(f"center x: {center_x}, center y: {center_y}")

            
            
            # self.get_logger().info("Recieved detections")
        
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
        self.extrin = cp.asarray(extrinsic[:3, :])
        self.extrinsic = cp.asarray(extrinsic)
        
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
            elif i < 4:
                line = line.split(' ')
                intrinsic[i-1]= [float(x) for x in line if x != '']
            else:
                line = line.split(' ')
                for j, dist in enumerate(line):
                    if dist != '':
                        distortion[j] = float(dist)
        self.intrinsic = cp.asarray(intrinsic)
        self.distortion = cp.asarray(distortion)
        file.close()
        self.get_logger().info('Loaded intrinsic data')

    def getTheoreticalUV(self, x, y, z, projected_points):
        # intrinsic is 3x3
        # extrinsic is 3x4
        # m3 4x1
        m3 = np.array([x, y, z, 1]) 
        result = np.matmul(self.intrinsic, self.extrin, m3.T)
        
        depth = result[2]
        u = result[0] / depth
        v = result[1] / depth

        return u, v


def main(args=None):
    rclpy.init(args=args)

    node = ProjectionNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()