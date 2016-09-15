#!/usr/bin/env python
import rospy

class MbedInterface(object):
    def __init__(self):
        rospy.init_node('mbed_interface')

    def run(self):
        rospy.spin()



if __name__ == "__main__":
	mbedInterface = MbedInterface()
	mbedInterface.run()
