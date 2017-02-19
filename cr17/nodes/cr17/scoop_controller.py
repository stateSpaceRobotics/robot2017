#!/usr/bin/env python2
import rospy
from std_msgs.msg import Int16


class ScoopController(object):
    def __init__(self):
        rospy.init_node('scoop_controller')

        self.autonomy = True

    def run(self):
        rospy.spin()

    def set_autonomy(self, autonomous):
        self.autonomy = autonomous


if __name__ == "__main__":
    scoopController = ScoopController()
    scoopController.run()
