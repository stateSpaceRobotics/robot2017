#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg       import Float64, Bool


CUR_ARM_ANGLE_TOPIC = rospy.get_param("topics/cur_arm_angle", "current_arm_angle")
DES_ARM_ANGLE_TOPIC = rospy.get_param("topics/des_arm_angle", "desired_arm_angle")
HAND_STATE_TOPIC = rospy.get_param("topics/hand_state", "hand_state")
TWIST_TOPIC = rospy.get_param("topics/drive_cmds", "cmd_vel")
JOY_TOPIC = rospy.get_param("topics/joystick", "joy")
LEFT_DRIVE_AXIS = rospy.get_param("joy/left_drive", 1)
RIGHT_DRIVE_AXIS = rospy.get_param("joy/right_drive", 4)
ARM_DOWN_AXIS = rospy.get_param("joy/arm_down", 2)
ARM_UP_AXIS = rospy.get_param("joy/arm_up", 5)
HAND_DOWN_BUTTON = rospy.get_param("joy/hand_down", 4)
HAND_UP_BUTTON = rospy.get_param("joy/hand_up", 5)
WHEEL_SEPARATION = rospy.get_param("wheel_separation", 1)
WHEEL_RADIUS = rospy.get_param("wheel_radius", 0.25)
ARM_ANGLE_UP = rospy.get_param("arm_up_angle", 100)
ARM_ANGLE_DOWN = rospy.get_param("arm_down_angle", 10)


class high_level_state_controller(object):
    def __init__(self):
        rospy.init_node("high_level_state_controller")

        self.drive_pub = rospy.Publisher(TWIST_TOPIC, Twist, queue_size=10)
        self.arm_pub = rospy.Publisher(DES_ARM_ANGLE_TOPIC, Float64, queue_size=10)
        self.hand_pub = rospy.Publisher(HAND_STATE_TOPIC, Bool, queue_size=10)
        self.handState = False

        rospy.Subscriber(CUR_ARM_ANGLE_TOPIC, Float64, self.arm_sub)
        self.arm_cur_angle = 45.0
        rospy.Subscriber(JOY_TOPIC, Joy, self.joy_sub)
        self.autoState = "INIT"
        self.highState = "TELE"#"AUTO"
        self.lastJoy = None
        self.leftDrive = None
        self.rightDrive = None
        self.armDown = None
        self.armUp = None
        self.handDown = None
        self.handUp = None

    def joy_sub(self, data):
        self.lastJoy = data.header.stamp
        self.leftDrive = data.axes[LEFT_DRIVE_AXIS]
        self.rightDrive = -data.axes[RIGHT_DRIVE_AXIS]
        self.armDown = data.axes[ARM_DOWN_AXIS]
        self.armUp = data.axes[ARM_UP_AXIS]
        self.handDown = data.buttons[HAND_DOWN_BUTTON]
        self.handUp = data.buttons[HAND_UP_BUTTON]

    def arm_sub(self, data):
        self.arm_cur_angle = data

    def linAngVelFromSkidSteer(self, left, right):
        x = (right+left)*WHEEL_RADIUS/2
        z = (right-left)*WHEEL_RADIUS/WHEEL_SEPARATION
        return x,z

    def cleanLeftJoy(self, leftIn):
        return leftIn

    def cleanRightJoy(self, rightIn):
        return -rightIn

    def combineCleanArm(self, armDown, armUp):
        return (-1*(armUp-1))+(armDown-1)

    def calcHandState(self, handUp, handDown, handState):
        if(handUp and not handDown):
            handState = True
        elif(handDown and not handUp):
            handState = False
        return handState

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #in autonomous mode
            if(self.highState == "AUTO"):
                #Act on the autoState
                pass

            elif(self.highState == "TELE"):
                if(self.lastJoy is not None):
                    cmd_vel = Twist()
                    left = self.cleanLeftJoy(self.leftDrive)
                    right = self.cleanRightJoy(self.rightDrive)
                    cmd_vel.linear.x, cmd_vel.angular.z = self.linAngVelFromSkidSteer(left, right)
                    self.drive_pub.publish(cmd_vel)

                    armCommand = self.combineCleanArm(self.armDown, self.armUp)
                    ArmMoveAmount = (ARM_ANGLE_UP - ARM_ANGLE_DOWN)/10
                    if(armCommand > .1):
                        self.arm_pub.publish(self.arm_cur_angle+ArmMoveAmount)
                    elif(armCommand < -0.1):
                        self.arm_pub.publish(self.arm_cur_angle-ArmMoveAmount)
                    else:
                        self.arm_pub.publish(self.arm_cur_angle)

                    self.handState = self.calcHandState(self.handUp, self.handDown, self.handState)
                    self.hand_pub.publish(self.handState)

            #advance the autostate
            
                
            rate.sleep()


if __name__ == "__main__":
    hlsc = high_level_state_controller()
    hlsc.run()