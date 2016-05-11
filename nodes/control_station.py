#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy


class control_station(object):
    def __init__(self):
        rospy.init_node("control_station")
        self.joy_pub = rospy.Publisher("joy_out", Joy, queue_size=10)
        rospy.Subscriber("joy", Joy, self.joy_sub)
        self.joy = Joy()

    def joy_sub(self, data):
        self.joy = data
        self.joy_pub.publish(data)

    def run(self):
        

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.joy_pub.publish(data)
            rate.sleep()

        #rospy.spin()


if __name__ == "__main__":
    ctrlStation = control_station()
    ctrlStation.run()
