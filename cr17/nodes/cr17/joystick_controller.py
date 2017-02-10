#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, String

'''
This module is used to convert joystick messages into appropriate affector commands.
'''

###################################
# Constants
CONTROLLER_BUTTONS = {"A": 0, "B":1, "X": 2, "Y": 3, "R1": 5, "L1": 4, "BACK": 6, "START": 7} # TODO: FINISH THIS, USE BELOW
CONTROLLER_AXES = {"LSTICKV": 1, "LSTICKH": 0}
# TODO: make these ROS parameters
MAX_MAG = 7
SLOP_THRESH = 0.15
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

        global DRIVE_SPEED
        try:
            # Constants for drive speeds
            DRIVE_SPEED = int(rospy.get_param("drive_settings/drive_speed"))
        except:
            rospy.logerr("Failed to load motor parameters.")  

        self.controller_state = Joy()

        self.prev_drive_state = Twist()

        self.teleopEnabled = None
        self.teleopButton_prev = 0
        if START_IN_TELEOP:
            self.endTeleop()
        else:
            self.startTeleop()

        # Load topic names
        self.joystick_topic       = rospy.get_param("topics/joystick", "joy")
        drive_topic               = rospy.get_param("topics/drive_cmds", "cmd_vel")
        # Setup publishers
        self.drive_pub = rospy.Publisher(drive_topic, Twist, queue_size = 10)
        # Setup subscribers
        rospy.Subscriber(self.joystick_topic, Joy, self.joy_callback)

    def startTeleop(self):
        #TODO: call the serives from navigator, scoop_controller, and the HLSC
        self.teleopEnabled = True

    def endTeleop(self):
        #TODO: call the serives from navigator, scoop_controller, and the HLSC
        self.teleopEnabled = False

    def joy_callback(self, data):
        '''
        Joy topic callback
        '''
        self.controller_state = data
        self.joy_received = True
        if(data.buttons[TELEOP_BUTTON] == 1):
            if(self.teleopButton_prev == 0):
                if self.teleopEnabled:
                    self.endTeleop()
                else:
                    self.startTeleop()
        self.teleopButton_prev = data.buttons[TELEOP_BUTTON]


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

            # mag = math.sqrt(lin_hist_avg**2 + vel_hist_avg**2)
            # if mag != 0:
            #     twister.linear.x = (lin_hist_avg / mag) * MAX_MAG
            #     twister.angular.z = (vel_hist_avg / mag) * MAX_MAG
            twister.linear.x = lin_hist_avg
            twister.angular.z = vel_hist_avg
            #if (twister.linear.x != self.prev_drive_state.linear.x) or (twister.angular.z != self.prev_drive_state.angular.z):
            self.drive_pub.publish(twister)
            self.prev_drive_state = twister

            self.joy_received = False
            rate.sleep()

    def _exit_handler(self):
        '''
        This function is called on exit 
        '''
        exit()

if __name__ == "__main__":
    controller = Joystick_Controller()
    controller.run()