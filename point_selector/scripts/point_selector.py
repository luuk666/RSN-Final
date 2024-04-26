#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped

class PointSelector:
    def __init__(self):
        rospy.init_node('pointcloud_interact', anonymous=True)
        self.point_pub = rospy.Publisher('/selected_point', PointStamped, queue_size=10)
        rospy.Subscriber("/pointcloud_topic", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/clicked_point", PointStamped, self.click_callback)

    def pointcloud_callback(self, data):
        pass

    def click_callback(self, msg):
        rospy.loginfo("Clicked point at: (%f, %f, %f)", msg.point.x, msg.point.y, msg.point.z)
        self.point_pub.publish(msg)

if __name__ == '__main__':
    selector = PointSelector()
    rospy.spin()