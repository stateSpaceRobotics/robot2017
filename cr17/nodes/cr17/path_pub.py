#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path

PATH_TOPIC = rospy.get_param("topics/path", "/obstacle_path")
POINT_1_X = rospy.get_param("path_points/dump_obstacle/x", -0.5)
POINT_1_Y = rospy.get_param("path_points/dump_obstacle/y", 1)
POINT_2_X = rospy.get_param("path_points/obstacle_mine/x", 0.5)
POINT_2_Y = rospy.get_param("path_points/obstacle_mine/y", 5)


class PathPublisher(object):
    def __init__(self):
        rospy.init_node("path_pubber")

        self.path_pub = rospy.Publisher(PATH_TOPIC, Path, queue_size = 10)

    def run(self):
        point1 = PoseStamped()
        point1.pose.position.x = POINT_1_X
        point1.pose.position.y = POINT_1_Y
        point2 = PoseStamped()
        point2.pose.position.x = POINT_2_X
        point2.pose.position.y = POINT_2_Y

        path = Path()
        path.poses = []
        path.poses.append(point1)
        path.poses.append(point2)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.path_pub.publish(path)
            rate.sleep()

        rospy.spin()


if __name__ == "__main__":
    path_pubber = PathPublisher()
    path_pubber.run()
