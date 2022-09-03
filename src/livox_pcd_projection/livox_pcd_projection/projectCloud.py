import rclpy
from rclpy.node import Node
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
    ObjectHypothesis
)
import cv2
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from livox_interfaces.msg import CustomMsg
from dist_msg.msg import Dist
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
        self.declare_parameter('bbox_size', 0.8)

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.intrinsic_path = self.get_parameter('intrinsic_path').get_parameter_value().string_value
        self.extrinsic_path = self.get_parameter('extrinsic_path').get_parameter_value().string_value
        self.lidar_threshold = self.get_parameter('lidar_threshold').get_parameter_value().integer_value
        self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().integer_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.bbox_size = self.get_parameter('bbox_size').get_parameter_value().double_value
        
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"Detection topic: {self.detection_topic}")
        self.get_logger().info(f"Lidar topic: {self.lidar_topic}")
        self.get_logger().info(f"intrinsic file path: {self.intrinsic_path}")
        self.get_logger().info(f"extrinsic file path: {self.extrinsic_path}")
        self.get_logger().info(f"lidar threshold: {self.lidar_threshold}")
        self.get_logger().info(f"node refresh rate: {self.refresh_rate}")
        self.get_logger().info(f"debug mode: {self.debug}")
        self.get_logger().info("Finished parsing parameters")

        self.load_extrinsic(self.extrinsic_path)
        self.load_intrinsic_distortion(self.intrinsic_path)

        self.lidar_decay_list = []

        self.get_logger().info("Starting message filter subscribers and registering callbacks")
        
        # create publisher for distances
        self.publisher = self.create_publisher(Dist, '/distances', 10)

        # create subscriber to bbox and pointcloud
        self.detection_sub = message_filters.Subscriber(self, Detection2DArray, self.detection_topic)
        self.lidar_sub = message_filters.Subscriber(self, CustomMsg, self.lidar_topic)

        if self.debug:
            # also subscribe to image for visualization purposes
            self.camera_sub = message_filters.Subscriber(self, Image, self.camera_topic)
            self.ts = message_filters.ApproximateTimeSynchronizer([self.camera_sub, self.detection_sub, self.lidar_sub], 10, 0.1)
            self.ts.registerCallback(self.debug_callback)
            self.debug_publisher = self.create_publisher(Image, '/debug_image', 10)
        else :
            self.ts = message_filters.ApproximateTimeSynchronizer([self.detection_sub, self.lidar_sub], 10, 1)
            self.ts.registerCallback(self.callback)

    def debug_callback(self, camera_msg, detection_msg, lidar_msg):
        projected_points = np.array([])
        bridge = CvBridge()
        pts_num = lidar_msg.point_num
        self.get_logger().info(f"Received {pts_num} points from lidar")
        cv_image = bridge.imgmsg_to_cv2(camera_msg, desired_encoding='rgb8')
        if len(self.lidar_decay_list) > self.lidar_threshold / pts_num + 1:
           self.lidar_decay_list.pop(0)
        self.lidar_decay_list.append(lidar_msg)
        x, y, z = [], [], []
        for i in range(len(self.lidar_decay_list)):
            for j in range(self.lidar_decay_list[i].point_num):
                x.append(self.lidar_decay_list[i].points[j].x)
                y.append(self.lidar_decay_list[i].points[j].y)
                z.append(self.lidar_decay_list[i].points[j].z)

            x_temp, y_temp, z_temp = np.array(x)[:,None], np.array(y)[:,None], np.array(z)[:,None]
            ones = np.ones((x_temp.shape[0], 1))
            points = cp.asarray(np.hstack((x_temp, y_temp, z_temp, ones)))
            if projected_points.size == 0:
                projected_points = self.getTheoreticalUV(x, points)
            else:
                projected_points = np.hstack((projected_points, self.getTheoreticalUV(x, points)))
            
        image = self.depth_to_color(projected_points, cv_image)  # Project the points to the image
        img_msg = bridge.cv2_to_imgmsg(image, encoding='rgb8')
        img_msg.header.stamp = camera_msg.header.stamp
        self.debug_publisher.publish(img_msg)

    

    def depth_to_color(self, projected_points, image, max_depth = 60, min_depth = 3):
        scale = (max_depth - min_depth) / 10
        depth = projected_points[0, :]
        for i in range(depth.shape[0]):
            if depth[i] < min_depth:
                r, g, b = 0, 0, 0xff
            elif depth[i] < min_depth + scale:
                r = 0
                g = int((depth[i] - min_depth) / scale * 255) & 0xff
                b = 0xff
            elif depth[i] < min_depth + 2 * scale:
                r = 0
                g = 0xff
                b = (0xff - int((depth[i] - min_depth - scale) / scale * 255)) & 0xff
            elif depth[i] < min_depth + 4 * scale:
                r = int((depth[i] - min_depth - scale*2) / scale * 255) & 0xff
                g = 0xff
                b = 0
            elif depth[i] < min_depth + 7 * scale:
                r = 0xff
                g = (0xff - int((depth[i] - min_depth - scale*4) / scale * 255)) & 0xff
                b = 0
            elif depth[i] < max_depth + 10 * scale:
                r = 0xff
                g = 0
                b = int((depth[i] - min_depth - scale*7) / scale * 255) & 0xff
            else:
                r, g, b = 0xff, 0, 0xff

            bgr = [r, g, b]
            u, v = projected_points[1, i], projected_points[2, i]
            image = cv2.circle(image, (int(u), int(v)), 1, bgr, -1)
        
        return image

            
    def callback(self, detection_msg, lidar_msg):
        pts_num = lidar_msg.point_num
        if len(self.lidar_decay_list) > self.lidar_threshold / pts_num + 1:
           self.lidar_decay_list.pop(0)
        self.lidar_decay_list.append(lidar_msg)

        # saving all the lidar points in to an array
        projected_points = np.array([])
        x, y, z = [], [], []
        for i in range(len(self.lidar_decay_list)):
            for j in range(self.lidar_decay_list[i].point_num):
                x.append(self.lidar_decay_list[i].points[j].x)
                y.append(self.lidar_decay_list[i].points[j].y)
                z.append(self.lidar_decay_list[i].points[j].z)

            x_temp, y_temp, z_temp = np.array(x)[:,None], np.array(y)[:,None], np.array(z)[:,None]
            ones = np.ones((x_temp.shape[0], 1))
            points = cp.asarray(np.hstack((x_temp, y_temp, z_temp, ones)))
            if projected_points.size == 0:
                projected_points = self.getTheoreticalUV(x, points)
            else:
                projected_points = np.hstack((projected_points, self.getTheoreticalUV(x, points)))

            x.clear()
            y.clear()
            z.clear()

        # calculate the average distance to the bounding boxes
        dist_msg = Dist()
        for detection in detection_msg.detections:
            # parse data from detection_msg
            dist_msg.obj_classes.append(detection.results[0].hypothesis.class_id)
            bbox = detection.bbox
            pose2d = bbox.center
            center_x, center_y = pose2d.x, pose2d.y
            width, height = bbox.size_x, bbox.size_y

            temp = []
            us = projected_points[1, :]
            vs = projected_points[2, :]
            right = center_x + width / 2 * self.bbox_size
            left = center_x - width / 2 * self.bbox_size
            top = center_y - height / 2 * self.bbox_size
            bottom = center_y + height / 2 * self.bbox_size
            temp = projected_points[0, (us >= left) & (us <= right) & (vs >= top) & (vs <= bottom)]

            temp = temp[~np.isnan(temp)]
            result = self.calc_dist(temp)
            dist_msg.distances.append(result)
        
        # publish Dist message
        dist_msg.header.stamp = self.get_clock().now().to_msg() # timestamp
        dist_msg.count = len(detection_msg.detections)
        self.publisher.publish(dist_msg)

    def calc_dist(self, dist_array):
        """Change this functiont to use different distance calculating methods"""
        return np.median(dist_array)
                                
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

    def getTheoreticalUV(self, x, points): # input is on GPU
        # intrinsic is 3x3
        # extrinsic is 3x4
        # m3 4x1
        result = cp.matmul(cp.matmul(self.intrinsic, self.extrin), points.T)
        
        depth = result[2,:]
        u = result[0,:] / depth
        v = result[1,:] / depth
        u, v = cp.asnumpy(u), cp.asnumpy(v)
        x = np.array(x)

        return np.vstack((x, u, v))


def main(args=None):
    rclpy.init(args=args)

    node = ProjectionNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()