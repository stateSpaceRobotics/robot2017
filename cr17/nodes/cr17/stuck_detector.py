#!/usr/bin/env python2
import rospy

class StuckDetector(object):
    def __init__(self):
        rospy.init_node('stuck_detector')

    def run(self):
        rospy.spin()



if __name__ == "__main__":
	stuckDetector = StuckDetector()
	stuckDetector.run()
