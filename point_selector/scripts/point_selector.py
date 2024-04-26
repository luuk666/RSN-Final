#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped

class PointSelector:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pointcloud_interact', anonymous=True)
        
        # 创建一个发布者，用于发布选中的点的坐标
        self.point_pub = rospy.Publisher('/selected_point', PointStamped, queue_size=10)
        
        # 订阅PointCloud2数据话题
        rospy.Subscriber("/pointcloud_topic", PointCloud2, self.pointcloud_callback)
        
        # 订阅rviz的点击点话题
        rospy.Subscriber("/clicked_point", PointStamped, self.click_callback)

    def pointcloud_callback(self, data):
        # 只是简单地保持节点活跃，监听点云数据
        pass

    def click_callback(self, msg):
        # 接收到点击的点的坐标并打印
        rospy.loginfo("Clicked point at: (%f, %f, %f)", msg.point.x, msg.point.y, msg.point.z)
        
        # 将选中的点坐标发布出去
        self.point_pub.publish(msg)

if __name__ == '__main__':
    selector = PointSelector()
    rospy.spin()