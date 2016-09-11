#!/usr/bin/env python
import rospy

class ObstacleMapper(object):
    def __init__(self):
        rospy.init_node('obstacle_mapper')

    def run(self):
        rospy.spin()



if __name__ == "__main__":
	obstacleMapper = ObstacleMapper()
	obstacleMapper.run()
