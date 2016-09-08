#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

CONTROLLER_BUTTONS = {"A": 0, "B":1, "X": 2, "Y": 3, "R1": 5, "L1": 4, "BACK": 6, "START": 7} # TODO: FINISH THIS, USE BELOW
CONTROLLER_AXES = {"LSTICKV": 1, "LSTICKH": 0}
# TODO: make this ROS parameters
MAX_MAG = 7
SLOP_THRESH = 0.15
# Axes to use for drive twists
JOY_LINEAR_AXIS = CONTROLLER_AXES["LSTICKV"]
JOY_ANGULAR_AXIS = CONTROLLER_AXES["LSTICKH"]

class joystick_controller(object):
    def __init__(self):
        rospy.init_node("joystick_controller")

        self.controllerState = None
        self.joyReceived = False

        # Load topic names
        self.joystick_topic = rospy.get_param("topics/joystick", "joy")
        drive_topic = rospy.get_param("topics/drive_cmds", "cmd_vel")

        # Setup publishers
        self.drive_pub = rospy.Publisher(drive_topic, Twist, queue_size = 10)
        # Setup subscribers
        rospy.Subscriber(self.joystick_topic, Joy, self.joy_callback)

    def joyCallback(self, data):
        '''
        Joy topic callback
        '''
        self.controllerState = data
        self.joyReceived = True

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
            current_state = self.controllerState
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
            self.drive_pub.publish(twister)
            self.prev_drive_state = twister

            self.joyReceived = False
            rate.sleep()

if __name__ == "__main__":
    JoystickController = joystick_controller()
    JoystickController.run()