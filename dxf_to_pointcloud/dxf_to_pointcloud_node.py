#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import ezdxf
import rosbag
import struct

def read_dxf(filename): #read dxf function
    doc = ezdxf.readfile(filename)
    modelspace = doc.modelspace()
    points = []
    for entity in modelspace:
        if entity.dxftype() == 'POINT':
            points.append([entity.dxf.location[0], entity.dxf.location[1], entity.dxf.location[2]])
    return points

def convert_to_pointcloud(points):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    # Calculate width and height of point cloud
    width = len(points)
    height = 1  # Assume for 1

    pointcloud_msg = PointCloud2()
    pointcloud_msg.header = header
    pointcloud_msg.fields = fields
    pointcloud_msg.is_bigendian = False
    pointcloud_msg.point_step = 12  # each point 4
    pointcloud_msg.row_step = width * pointcloud_msg.point_step
    pointcloud_msg.is_dense = True

    # pack the coordinates of the point and put them into the data field
    pointcloud_msg.data = []
    for point in points:
        for coord in point:
            pointcloud_msg.data += struct.pack('f', coord)

    # Set data
    pointcloud_msg.width = width
    pointcloud_msg.height = height

    return pointcloud_msg

def main():
    rospy.init_node('dxf_to_pointcloud', anonymous=True)
    pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)

    dxf_file = '/home/edba6/point_cloud.dxf'
    points = read_dxf(dxf_file)
    pointcloud_msg = convert_to_pointcloud(points)

    rate = rospy.Rate(1)  # set for 1Hz
    with rosbag.Bag('pointcloud.bag', 'w') as bag:
        while not rospy.is_shutdown():
            pointcloud_msg.header.stamp = rospy.Time.now()
            pub.publish(pointcloud_msg)
            bag.write('/pointcloud', pointcloud_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass