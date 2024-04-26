#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d

def callback(data):
    points = []

    # Read message
    for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        points.append([point[0], point[1], point[2]])

    # Create
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Visualize
    o3d.visualization.draw_geometries([pcd], window_name="PointCloud Viewer")

def pointcloud_viewer_node():
    rospy.init_node('pointcloud_viewer_node')
    rospy.Subscriber("/pointcloud", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    pointcloud_viewer_node()
