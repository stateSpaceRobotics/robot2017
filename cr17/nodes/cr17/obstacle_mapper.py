#!/usr/bin/env python2
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import MapMetaData, OccupancyGrid


class ObstacleMapper(object):
    def __init__(self):
        rospy.init_node('obstacle_mapper')
        rospy.Subscriber('robot_pose', Pose, self.robot_pose_cb)
        rospy.Subscriber('obstacles', PointCloud2, self.obstacles_cb)
        self.robot_pose = Pose
        self.obstacles = PointCloud2

        self.pub = rospy.Publisher('map', OccupancyGrid, latch=True, queue_size=5)

    def robot_pose_cb(self, data):
        rospy.loginfo("Got robot_pose:\n%s", data)
        self.robot_pose = data

    def obstacles_cb(self, data):
        rospy.loginfo("Got obstacles:\n%s", data)
        self.obstacles = data

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            h = Header()
            h.stamp = rospy.Time.now()
            self.pub.publish(OccupancyGrid(header=h))
            rate.sleep()

if __name__ == "__main__":
    obstacleMapper = ObstacleMapper()
    obstacleMapper.run()
