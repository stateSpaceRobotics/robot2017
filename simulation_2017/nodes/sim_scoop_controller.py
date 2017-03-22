#!/usr/bin/env python2
import rospy, math
from std_msgs.msg import Float64
from cr17.msg import scoopControl

# Topics
ARM_STATE_TOPIC = rospy.get_param('topics/scoop_state_cmds')
ARM_GOAL_TOPIC = rospy.get_param('topics/scoop_cmds')

# Scooper/Arm Angles
DRIVING_ARM_ANGLE = rospy.get_param('scoop_config/driving_arm_angle')
DRIVING_SCOOP_ANGLE = rospy.get_param('scoop_config/driving_scoop_angle')

PRE_DIG_ARM_ANGLE = rospy.get_param('scoop_config/pre_dig_arm_angle')
PRE_DIG_SCOOP_ANGLE = rospy.get_param('scoop_config/pre_dig_scoop_angle')

DIGGING_ARM_ANGLE = rospy.get_param('scoop_config/digging_arm_angle')
DIGGING_SCOOP_ANGLE = rospy.get_param('scoop_config/digging_scoop_angle')

POST_DIG_ARM_ANGLE = rospy.get_param('scoop_config/post_dig_arm_angle')
POST_DIG_SCOOP_ANGLE = rospy.get_param('scoop_config/post_dig_scoop_angle')

PRE_DUMP_ARM_ANGLE = rospy.get_param('scoop_config/pre_dump_arm_angle')
PRE_DUMP_SCOOP_ANGLE = rospy.get_param('scoop_config/pre_dump_scoop_angle')

DUMPING_ARM_ANGLE = rospy.get_param('scoop_config/dumping_arm_angle')
DUMPING_SCOOP_ANGLE = rospy.get_param('scoop_config/dumping_scoop_angle')

POST_DUMP_ARM_ANGLE = rospy.get_param('scoop_config/post_dump_arm_angle')
POST_DUMP_SCOOP_ANGLE = rospy.get_param('scoop_config/post_dump_scoop_angle')


class ScoopController(object):
    def __init__(self):
        rospy.init_node('sim_scoop_controller')

        # What should be the starting values for these
        self.autonomous = True
        self.scoop_msg = scoopControl()
        self.defined_msgs = []
        self.create_defined_msgs()

        # ROS Subscribers
        self.arm_sub = rospy.Subscriber(ARM_STATE_TOPIC, scoopControl, self.update_scoop_msg, queue_size=10)

        # ROS Publishers
        self.left_arm_pub = rospy.Publisher("/cr17_left_arm_controller/command", Float64, queue_size=10)
        self.left_scoop_pub = rospy.Publisher("/cr17_left_scoop_controller/command", Float64, queue_size=10)
        self.right_arm_pub = rospy.Publisher("/cr17_right_arm_controller/command", Float64, queue_size=10)
        self.right_scoop_pub = rospy.Publisher("/cr17_right_scoop_controller/command", Float64, queue_size=10)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.autonomous:
                # Custom Angles
                if self.scoop_msg.desiredState == self.scoop_msg.state_customAngles:
                    self.left_arm_pub.publish(self.scoop_msg.armAngle/180.0)
                    self.left_scoop_pub.publish(self.scoop_msg.scoopAngle/180.0)
                    self.right_arm_pub.publish(self.scoop_msg.armAngle/180.0)
                    self.right_scoop_pub.publish(self.scoop_msg.scoopAngle/180.0)
                else:
                    self.left_arm_pub.publish(self.defined_msgs[self.scoop_msg.desiredState].armAngle/180.0)
                    self.left_scoop_pub.publish(self.defined_msgs[self.scoop_msg.desiredState].scoopAngle/180.0)
                    self.right_arm_pub.publish(self.defined_msgs[self.scoop_msg.desiredState].armAngle/180.0)
                    self.right_scoop_pub.publish(self.defined_msgs[self.scoop_msg.desiredState].scoopAngle/180.0)
            rate.sleep()

    def update_scoop_msg(self, msg):
        self.scoop_msg = msg

    def create_defined_msgs(self):
        for state_msg in range(0, 8):
            new_msg = scoopControl()
            new_msg.desiredState = state_msg
            if state_msg == new_msg.state_drivingPosition:
                new_msg.armAngle = DRIVING_ARM_ANGLE
                new_msg.scoopAngle = DRIVING_SCOOP_ANGLE
            elif state_msg == new_msg.state_preDigging:
                new_msg.armAngle = PRE_DIG_ARM_ANGLE
                new_msg.scoopAngle = PRE_DIG_SCOOP_ANGLE
            elif state_msg == new_msg.state_digging:
                new_msg.armAngle = DIGGING_ARM_ANGLE
                new_msg.scoopAngle = DIGGING_SCOOP_ANGLE
            elif state_msg == new_msg.state_postDigging:
                new_msg.armAngle = POST_DIG_ARM_ANGLE
                new_msg.scoopAngle = POST_DIG_SCOOP_ANGLE
            elif state_msg == new_msg.state_preDump:
                new_msg.armAngle = PRE_DUMP_ARM_ANGLE
                new_msg.scoopAngle = PRE_DUMP_SCOOP_ANGLE
            elif state_msg == new_msg.state_dump:
                new_msg.armAngle = DUMPING_ARM_ANGLE
                new_msg.scoopAngle = DUMPING_SCOOP_ANGLE
            elif state_msg == new_msg.state_postDump:
                new_msg.armAngle = POST_DUMP_ARM_ANGLE
                new_msg.scoopAngle = POST_DUMP_SCOOP_ANGLE
            self.defined_msgs.append(new_msg)


if __name__ == '__main__':
    scoopController = ScoopController()
    scoopController.run()