#!/usr/bin/env python2
import rospy

class ScoopController(object):
    def __init__(self):
        rospy.init_node('scoop_controller')

    def run(self):
        rospy.spin()



if __name__ == "__main__":
	scoopController = ScoopController()
	scoopController.run()
