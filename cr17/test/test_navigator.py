#!/usr/bin/env python2

PKG='cr17'
NAME='test_navigator'

import unittest
import rostest

from cr17 import navigator
from geometry_msgs.msg import Point, Pose



class TestNavigator(unittest.TestCase):
    def assertListAlmostEqual(self, list1, list2, tol = 7):
        self.assertEqual(len(list1), len(list2))
        for a, b in zip(list1, list2):
             self.assertAlmostEqual(a, b, tol)

    def test_at_goal(self):
        samePose = Point()
        differentPose = Point()
        robotPose = Pose()

        samePose.x = 1
        samePose.y = 12

        differentPose.x = 12
        differentPose.y = 1

        robotPose.position.x = 1
        robotPose.position.y = 12

        self.assertTrue(navigator.at_goal(robotPose, samePose))
        self.assertFalse(navigator.at_goal(robotPose, differentPose))

    def test_calc_goal_force(self):
        goalForward = Point()
        goalBackward = Point()
        goalLeft = Point()
        goalRight = Point()
        goalFrontLeft = Point()
        goalFrontRight = Point()
        goalBackLeft = Point()
        goalBackRight = Point()

        goalForward.x = 1
        goalForward.y = 0

        goalBackward.x = -1
        goalBackward.y = 0

        goalLeft.x = 0
        goalLeft.y = 1

        goalRight.x = 0
        goalRight.y = -1

        goalFrontLeft.x = 1
        goalFrontLeft.y = 1

        goalFrontRight.x = 1
        goalFrontRight.y = -1

        goalBackLeft.x = -1
        goalBackLeft.y = 1

        goalBackRight.x = -1
        goalBackRight.y = -1

        robotPose = Pose()
        robotPose.position.x = 0
        robotPose.position.y = 0

        self.assertListAlmostEqual((0.9,0), navigator.calc_goal_force(goalForward, robotPose))
        self.assertListAlmostEqual((-0.9,0), navigator.calc_goal_force(goalBackward, robotPose))
        self.assertListAlmostEqual((0,0.9), navigator.calc_goal_force(goalLeft, robotPose))
        self.assertListAlmostEqual((0,-0.9), navigator.calc_goal_force(goalRight, robotPose))
        self.assertListAlmostEqual((0.929,0.929), navigator.calc_goal_force(goalFrontLeft, robotPose), 3)
        self.assertListAlmostEqual((0.929,-0.929), navigator.calc_goal_force(goalFrontRight, robotPose), 3)
        self.assertListAlmostEqual((-0.929,0.929), navigator.calc_goal_force(goalBackLeft, robotPose), 3)
        self.assertListAlmostEqual((-0.929,-0.929), navigator.calc_goal_force(goalBackRight, robotPose), 3)

    def test_calc_repulsive_force(self):
        obstacle1 = (0,0.4)
        obstacle2 = (0,-0.4)
        obstacle3 = (0.4,0)
        obstacle4 = (-0.4,0)

        robotPose = Pose()
        robotPose.position.x = 0
        robotPose.position.y = 0

        self.assertListAlmostEqual((0,-0.11), navigator.calc_repulsive_force([obstacle1],robotPose))
        self.assertListAlmostEqual((0,0.11), navigator.calc_repulsive_force([obstacle2],robotPose))
        self.assertListAlmostEqual((-0.11,0), navigator.calc_repulsive_force([obstacle3],robotPose))
        self.assertListAlmostEqual((0.11,0), navigator.calc_repulsive_force([obstacle4],robotPose))

        self.assertListAlmostEqual((0,-0.22), navigator.calc_repulsive_force([obstacle1, obstacle1],robotPose))
        self.assertListAlmostEqual((0,0), navigator.calc_repulsive_force([obstacle1, obstacle2],robotPose))
        self.assertListAlmostEqual((-0.11,-0.11), navigator.calc_repulsive_force([obstacle1, obstacle3],robotPose))
        self.assertListAlmostEqual((0.11,-0.11), navigator.calc_repulsive_force([obstacle1, obstacle4],robotPose))

        self.assertListAlmostEqual((0,0), navigator.calc_repulsive_force([obstacle2, obstacle1],robotPose))
        self.assertListAlmostEqual((0, 0.22), navigator.calc_repulsive_force([obstacle2, obstacle2],robotPose))
        self.assertListAlmostEqual((-0.11, 0.11), navigator.calc_repulsive_force([obstacle2, obstacle3],robotPose))
        self.assertListAlmostEqual((0.11, 0.11), navigator.calc_repulsive_force([obstacle2, obstacle4],robotPose))
        
        self.assertListAlmostEqual((-0.11,-0.11), navigator.calc_repulsive_force([obstacle3, obstacle1],robotPose))
        self.assertListAlmostEqual((-0.11, 0.11), navigator.calc_repulsive_force([obstacle3, obstacle2],robotPose))
        self.assertListAlmostEqual((-0.22, 0), navigator.calc_repulsive_force([obstacle3, obstacle3],robotPose))
        self.assertListAlmostEqual((0,0), navigator.calc_repulsive_force([obstacle3, obstacle4],robotPose))

        self.assertListAlmostEqual((0.11,-0.11), navigator.calc_repulsive_force([obstacle4, obstacle1],robotPose))
        self.assertListAlmostEqual((0.11, 0.11), navigator.calc_repulsive_force([obstacle4, obstacle2],robotPose))
        self.assertListAlmostEqual((0,0), navigator.calc_repulsive_force([obstacle4, obstacle3],robotPose))
        self.assertListAlmostEqual((0.22, 0), navigator.calc_repulsive_force([obstacle4, obstacle4],robotPose))


if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestNavigator)
