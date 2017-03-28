#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan


class LidarCoalescer(object):
    def __init__(self):
        rospy.init_node('lidar_coalescer')

        self.hokuyo_lidar_topic = rospy.get_param("topics/lidar/hokuyo")
        self.sick_lidar_topic = rospy.get_param("topics/lidar/sick")
        self.scan_topic = rospy.get_param("beacon_localization/scan_topic")

        # ROS Subscribers
        self.hokuyo_sub = rospy.Subscriber(self.hokuyo_lidar_topic, LaserScan, self.hokuyo_callback, queue_size=10)
        self.sick_sub = rospy.Subscriber(self.sick_lidar_topic, LaserScan, self.sick_callback, queue_size=10)

        # ROS Publishers
        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=10)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def hokuyo_callback(self):
        return

    def sick_callback(self):
        return

if __name__ == "__main__":
    lidar_coalescer = LidarCoalescer()
    lidar_coalescer.run()
