#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from gray_transceiver.msg import GxRequest, GxMetaTopic

class joy_repubber(object):
    def __init__(self):
        rospy.init_node("joy_repubber")
        self.joy_pub = rospy.Publisher("joy", Joy, queue_size = 10)
        rospy.Subscriber("gray_transceiver/metatopic", GxMetaTopic, self.meta_sub)
        self.subscribers = []

        self.request_pub = rospy.Publisher("gray_transceiver/requests", GxRequest, queue_size=10)
        

    def joy_sub(self, data):
        self.joy_pub.publish(data)

    def meta_sub(self, data):
        self.thisRobotName = data.myName
        if(data.type == "sensor_msgs/Joy"):
            self.subscribers.append(rospy.Subscriber(data.name, Joy, self.joy_sub))


    def run(self):
        rate = rospy.Rate(1)
        for each in range(0,10):
            message = GxRequest()
            message.description = "joy"
            message.type = "sensor_msgs/Joy"
            self.request_pub.publish(message)
            rate.sleep()
        rospy.spin()


if __name__ == "__main__":
    ctrlStation = joy_repubber()
    ctrlStation.run()
