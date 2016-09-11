#!/usr/bin/env python
import rospy

class PathPlanner(object):
    def __init__(self):
        rospy.init_node('path_planner')

    def run(self):
        rospy.spin()



if __name__ == "__main__":
	pathPlanner = PathPlanner()
	pathPlanner.run()
