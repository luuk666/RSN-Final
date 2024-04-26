#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
import networkx as nx
from networkx.algorithms.approximation import traveling_salesman_problem
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class PathPlanner:
    def __init__(self):
        self.graph = nx.Graph()
        self.points = []
        
        rospy.init_node('path_planner')
        rospy.Subscriber('/selected_point', PointStamped, self.point_callback)
        self.path_pub = rospy.Publisher('/path', Marker, queue_size=10)

    def point_callback(self, msg):
        point = (msg.point.x, msg.point.y)  # 只考虑x和y坐标
        if not self.points:
            self.start_z = msg.point.z  # 存储第一个点的z坐标
        if point not in self.points:
            self.points.append(point)
            self.add_and_calculate_path()

    def add_and_calculate_path(self):
        if len(self.points) > 1:
            # 为图添加边，边的权重是欧几里得距离
            for i in range(len(self.points)-1):
                for j in range(i + 1, len(self.points)):
                    p1, p2 = self.points[i], self.points[j]
                    weight = self.euclidean_distance(p1, p2)
                    self.graph.add_edge(p1, p2, weight=weight)
            # 计算并发布最短路径
            self.calculate_and_publish_path()

    def calculate_and_publish_path(self):
        if len(self.points) > 2:
            path = traveling_salesman_problem(self.graph, cycle=True)
            print(path)
            self.publish_path(path)

    def euclidean_distance(self, p1, p2):
        distance = ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
        return distance

    def publish_path(self, path):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # 线宽
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # 红色

        for p in path:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = self.start_z   # Z坐标设置为0
            marker.points.append(point)

        self.path_pub.publish(marker)

if __name__ == '__main__':
    planner = PathPlanner()
    rospy.spin()