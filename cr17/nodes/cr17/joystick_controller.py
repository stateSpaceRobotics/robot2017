#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, String
from cr17.srv import autonomousActive
from cr17.msg import scoopControl


'''
This module is used to convert joystick messages into appropriate affector commands.
'''

###################################
# Constants
CONTROLLER_BUTTONS = {"A": 0, "B":1, "X": 2, "Y": 3, "R1": 5, "L1": 4, "BACK": 6, "START": 7} # TODO: FINISH THIS, USE BELOW
CONTROLLER_AXES = {"LSTICKV": 1, "LSTICKH": 0, "RTRIGGER":5, "LTRIGGER":2}
# TODO: make these ROS parameters
MAX_MAG = 1
SLOP_THRESH = 0.15
ARM_MOVE_MAG = 25
HAND_MOVE_MAG = 25
# Axes to use for drive twists
JOY_LINEAR_AXIS = CONTROLLER_AXES["LSTICKV"]
JOY_ANGULAR_AXIS = CONTROLLER_AXES["LSTICKH"]

ARM_DOWN_AXIS = rospy.get_param("joy/arm_down", 2)
ARM_UP_AXIS = rospy.get_param("joy/arm_up", 5)
HAND_DOWN_BUTTON = rospy.get_param("joy/hand_down", 4)
HAND_UP_BUTTON = rospy.get_param("joy/hand_up", 5)
TELEOP_BUTTON = rospy.get_param("joy/teleop", 8) #the center button on a wired xbox 360 controller

###################################
# Global Constants
# Drive constants
DRIVE_SPEED = 7
###################################

START_IN_TELEOP = rospy.get_param("init_in_teleop", True)


class Joystick_Controller(object):
    '''
    Drive: Twist messages
    '''
    def __init__(self):
        '''
        Joystick Controller constructor
        '''
        rospy.init_node("joystick_controller")

        self.joy_received = False
        self.arm_state = scoopControl()
        self.arm_state.armAngle = 50
        self.arm_state.scoopAngle = 50

        global DRIVE_SPEED
        try:
            # Constants for drive speeds
            DRIVE_SPEED = int(rospy.get_param("drive_settings/drive_speed", 1))
        except: 
            rospy.logerr("Failed to load motor parameters.")  

        self.controller_state = Joy()

        self.teleopEnabled = None
        self.teleopButton_prev = 0
        if START_IN_TELEOP:
            self.start_teleop()
        else:
            self.end_teleop()

        # Load topic names
        self.joystick_topic       = rospy.get_param("topics/joystick", "joy")
        drive_topic               = rospy.get_param("topics/drive_cmds", "cmd_vel")
        self.arm_state_topic      = rospy.get_param('topics/scoop_state_cmds', "cmd_scoop")
        self.arm_state_in         = rospy.get_param("topics/scoop_state_current", "scoop_out")

        # Setup publishers
        self.drive_pub = rospy.Publisher(drive_topic, Twist, queue_size = 10)
        self.scoop_pub = rospy.Publisher(self.arm_state_topic, scoopControl, queue_size=10)

        # Setup subscribers
        rospy.Subscriber(self.joystick_topic, Joy, self.joy_callback)
        rospy.Subscriber(self.arm_state_in, scoopControl, self.arm_callback)

    def start_teleop(self):
        self.teleopEnabled = True
        # try:
        #     autonomousScoopService = rospy.ServiceProxy("/autonomousScoop", autonomousActive)
        #     autonomousScoopService(False)
        # except rospy.ServiceException, e:
        #     rospy.logerr("/autonomousScoop service call failed: %s",e)

        try:
            autonomousNavigatorService = rospy.ServiceProxy("/autonomousNavigator", autonomousActive)
            autonomousNavigatorService(False)
        except rospy.ServiceException, e:
            rospy.logerr("/autonomousNavigator service call failed: %s",e)

        try:
            autonomousHLSCService = rospy.ServiceProxy("/autonomousHLSC", autonomousActive)
            autonomousHLSCService(False)
        except rospy.ServiceException, e:
            rospy.logerr("/autonomousHLSC service call failed: %s",e)

    def end_teleop(self):
        self.teleopEnabled = False
        try:
            autonomousScoopService = rospy.ServiceProxy("/autonomousScoop", autonomousActive)
            autonomousScoopService(True)
        except rospy.ServiceException, e:
            rospy.logerr("/autonomousScoop service call failed: %s",e)

        try:
            autonomousNavigatorService = rospy.ServiceProxy("/autonomousNavigator", autonomousActive)
            autonomousNavigatorService(True)
        except rospy.ServiceException, e:
            rospy.logerr("/autonomousNavigator service call failed: %s",e)

        try:
            autonomousHLSCService = rospy.ServiceProxy("/autonomousHLSC", autonomousActive)
            autonomousHLSCService(True)
        except rospy.ServiceException, e:
            rospy.logerr("/autonomousHLSC service call failed: %s",e)

    def joy_callback(self, data):
        '''
        Joy topic callback
        '''
        self.controller_state = data
        self.joy_received = True
        if(data.buttons[TELEOP_BUTTON] == 1):
            if(self.teleopButton_prev == 0):
                if self.teleopEnabled:
                    self.end_teleop()
                else:
                    self.start_teleop()
        self.teleopButton_prev = data.buttons[TELEOP_BUTTON]

    def arm_callback(self, data):
        self.arm_state = data

    def combineCleanArm(self, armDown, armUp):
        return (-1*(armUp-1))+(armDown-1)

    def run(self):
        '''
        This function is the processing function for this module.
        '''
        rospy.wait_for_message(self.joystick_topic, Joy)  # Wait for messege on joy topic
        rate = rospy.Rate(10)
        lin_history = [0 for i in xrange(0, 5)]
        vel_history = [0 for i in xrange(0, 5)]

        while not rospy.is_shutdown():
            # Grab most recent controller state
            current_state = self.controller_state
            self.joy_received = False
            ######
            # Build Twist message
            ######
            twister = Twist()
            #####
            # Get drive command
            #####
            lin_val = current_state.axes[JOY_LINEAR_AXIS]
            ang_val = current_state.axes[JOY_ANGULAR_AXIS]

            mag = math.sqrt(lin_val**2 + ang_val**2)
            if (lin_val <= SLOP_THRESH) and (lin_val >= -SLOP_THRESH):
                # Within 0 point slop
                lin_vel = 0
            else:   
                lin_vel = (lin_val / mag) * MAX_MAG

            if (ang_val <= SLOP_THRESH) and (ang_val >= -SLOP_THRESH):
                # Within 0 point slop
                ang_vel = 0
            else:   
                ang_vel = (ang_val / mag) * MAX_MAG

            lin_history.pop(0)
            vel_history.pop(0)
            lin_history.append(lin_vel)
            vel_history.append(ang_vel)

            lin_hist_avg = sum(lin_history) / len(lin_history)
            vel_hist_avg = sum(vel_history) / len(vel_history)

            twister.linear.x = lin_hist_avg     #reduce sudden changes
            twister.angular.z = vel_hist_avg    #reduce sudden changes
            if self.teleopEnabled:
                self.drive_pub.publish(twister)

            arm_up = current_state.axes[ARM_UP_AXIS]
            arm_down = current_state.axes[ARM_DOWN_AXIS]

            hand_up = current_state.buttons[HAND_UP_BUTTON]
            hand_down = current_state.buttons[HAND_DOWN_BUTTON]

            arm_change = self.combineCleanArm(arm_down, arm_up)
            hand_change = hand_up - hand_down

            try:
                arm_change = arm_change/abs(arm_change) #this is temporary and just for the video
            except Exception:
                arm_change = 0

            desired_scoop_state = scoopControl()

            desired_scoop_state.armAngle = self.arm_state.armAngle + (ARM_MOVE_MAG*arm_change)
            desired_scoop_state.scoopAngle = self.arm_state.scoopAngle + (HAND_MOVE_MAG*hand_change)

            self.scoop_pub.publish(desired_scoop_state)

            rate.sleep()

if __name__ == "__main__":
    controller = Joystick_Controller()
    controller.run()
