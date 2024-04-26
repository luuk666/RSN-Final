#!/usr/bin/env python3
import rospy
import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RGBDToPointCloud:
    def __init__(self):
        rospy.init_node('rgbd_to_pointcloud', anonymous=True)
        self.bridge = CvBridge()
        self.bag_path = '/home/edba6/rgbd_dataset_freiburg2_desk.bag' 
        self.process_bag()

    def process(self):
        with rosbag.Bag(self.bag_path, 'r') as bag:
            start_time = bag.get_start_time()
            end_time = bag.get_end_time()
            target_time = rospy.Time((start_time + end_time) / 2)  # Take the middle time of rosbag
            
            depth_image = None
            rgb_image = None
            found = False

            for topic, msg, t in bag.read_messages():
                if found or rospy.Time.from_sec(t.to_sec()) > target_time:
                    break
                if rospy.Time.from_sec(t.to_sec()) >= target_time - rospy.Duration(1) and rospy.Time.from_sec(t.to_sec()) <= target_time + rospy.Duration(1):
                    if topic == '/camera/depth/image':
                        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    elif topic == '/camera/rgb/image_color':
                        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    if depth_image is not None and rgb_image is not None:
                        found = True

            if found:
                print("Generating")
                points, colors = self.convert_to_pointcloud(rgb_image, depth_image)
                print("Displaying")
                self.display_pointcloud(points, colors)
            else:
                print("No image data")

    def convert_to_pointcloud(self, rgb_image, depth_image):
        points = []
        colors = []
        # Set kinect camera parameters
        fx = 525.0  
        fy = 525.0  
        cx = 320.0  
        cy = 240.0 
        for v in range(depth_image.shape[0]):
            for u in range(depth_image.shape[1]):
                Z = depth_image[v, u] / 1000.0  # Convert from millimeters to meters
                if Z > 0.0:
                    X = (u - cx) * Z / fx
                    Y = (v - cy) * Z / fy
                    points.append([X, Y, Z])
                    color = rgb_image[v, u]
                    colors.append(color)
        return np.array(points), np.array(colors)


    def display_pointcloud(self, points, colors):   # Show img
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors/255, marker='.')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

if __name__ == '__main__':
    try:
        rgbd_to_pointcloud = RGBDToPointCloud()
    except rospy.ROSInterruptException:
        pass
