#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Bool

PATH_TOPIC = rospy.get_param("topics/path", "/obstacle_path")
POINT_1_X = rospy.get_param("path_points/obstacle_mine/x", 0.75)
POINT_1_Y = rospy.get_param("path_points/obstacle_mine/y", 5)
POINT_2_X = rospy.get_param("path_points/dump_obstacle/x", -0.5)
POINT_2_Y = rospy.get_param("path_points/dump_obstacle/y", 1)

class PathPublisher(object):
    def __init__(self):
        rospy.init_node("path_pubber")

        self.path = Path()
        self.path_pub = rospy.Publisher(PATH_TOPIC, Path, queue_size = 10)

    def run(self):
        point1 = PoseStamped()
        point1.pose.position.x = POINT_1_X
        point1.pose.position.y = POINT_1_Y
        point1.pose.orientation.x = 0
        point1.pose.orientation.y = 0
        point1.pose.orientation.z = -0.7071067811865476
        point1.pose.orientation.w = -0.7071067811865476

        point2 = PoseStamped()
        point2.pose.position.x = POINT_2_X
        point2.pose.position.y = POINT_2_Y
        point2.pose.orientation.x = 0
        point2.pose.orientation.y = 0
        point2.pose.orientation.z = 0.7071067811865476
        point2.pose.orientation.w = -0.7071067811865476

        self.path.poses = []
        self.path.poses.append(point1)
        self.path.poses.append(point2)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.path_pub.publish(self.path)
            rate.sleep()

        rospy.spin()

if __name__ == "__main__":
    path_pubber = PathPublisher()
    path_pubber.run()
