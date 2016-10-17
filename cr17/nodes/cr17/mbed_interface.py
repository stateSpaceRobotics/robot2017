#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from cr17.msg import wheelData, scoopControl

import os
import sys
 
import usb.core
import usb.util
 
from time import sleep
import random

'''
Sends low-level commands(scoop, arm, linear velocity, angular velocity) to Mbed over USB.
Also accepts speed values from Mbed for each wheel, and publishes to the wheel_speed topic. 
'''

##Mbed Send Package Description(8 bytes)
#0 CMD_Vel: Z-Angular
#1 CMD_Vel: X-inear
#2 Scoop  : Arm-Angular
#3 Scoop  : Scoop-Angular
#4 Unused :
#5 Unused :
#6 Unused :
#7 Unused :

ROS_SLEEP_RATE = 10

MBED_VENDOR_ID = 0x1234
MBED_PRODUCT_ID = 0x0006

# 4 bits for fraction + 3 bits for integer + 1 bit for sign = 8 bits!!! ()
FRAC_BIT_WIDTH = 4
INT_BIT_WIDTH = 3

SIGN_BYTE = 0b10000000

UPPER_LIMIT = 7.9
LOWER_LIMIT = -7.9
######################################
#Topic Variables
######################################
DRIVE_TOPIC = "/cmd_vel"     
SCOOP_TOPIC = "/cmd_scoop"   
WHEEL_SPEED_TOPIC = "/wheel_speed"


class MbedInterface(object):
    def __init__(self):
        rospy.init_node('mbed_interface')
        
        #These will be updated as new data is received for it.
        self.__cmd_vel = Twist()
        self.__cmd_scoop = scoopControl()
        self.__wheel_data = wheelData()

        #Raw data list(byte array?) to/from the Mbed
        self.__data = [0x0] * 8  #Initializes bytes to be set to zero
        self.__bytes = None
        
        ######################################
        # Setup ROS publishers
        ######################################
        rospy.Subscriber(DRIVE_TOPIC, Twist, self.cmd_vel_callback)
        rospy.Subscriber(SCOOP_TOPIC, scoopControl, self.cmd_scoop_callback)


        ######################################
        # Setup ROS publishers
        ######################################
        self.wheel_speed_pub = rospy.Publisher(WHEEL_SPEED_TOPIC, Twist, queue_size = 10)

        # Find device
        self.__hid_device = usb.core.find(idVendor=MBED_VENDOR_ID,idProduct=MBED_PRODUCT_ID)
    
        if not self.__hid_device:
            print "No device connected"
        else:
            sys.stdout.write('mbed found\n')
            if self.__hid_device.is_kernel_driver_active(0):
                try:
                    self.__hid_device.detach_kernel_driver(0)
                except usb.core.USBError as e:
                    sys.exit("Could not detatch kernel driver: %s" % str(e))
            try:
                self.__hid_device.set_configuration()
                self.__hid_device.reset()
            except usb.core.USBError as e:
                sys.exit("Could not set configuration: %s" % str(e))
            
            self.__endpoint = self.__hid_device[0][(0,0)][0]



    ######################################
    # ROS Callbacks
    ######################################

    #Add new cmd_vel to USB message(x-linear/z-angular)
    def cmd_vel_callback(self, twist_msg):
        print "Received Twist from ", DRIVE_TOPIC
        self.__data[0] = self.float_to_fixed_point(twist_msg.angular.z)
        self.__data[1] = self.float_to_fixed_point(twist_msg.linear.x)

    #Add new scoop cmd to USB message(arm velocity/scoop velocity)
    def cmd_scoop_callback(self, scoop_msg):
        print "Received scoopControl from ", SCOOP_TOPIC
        self.__data[2] = self.float_to_fixed_point(scoop_msg.armVelAngular)
        self.__data[3] = self.float_to_fixed_point(scoop_msg.scoopVelAngular)

    #Sends data recieved from Mbed over a topic(wheelData)
    def mbed_recieve_handler(self, data):
        print "Mbed Recv: ", data

    ######################################
    # Needed Functions
    ######################################

    #Converts float to fixed point(represented as an 8 byte int) based of the scheme described in the wiki.
    def float_to_fixed_point(self, float_val):

        #Raises an error if values are out of the limts of what this unibyte encoding can handle
        if (float_val > UPPER_LIMIT) | (float_val < LOWER_LIMIT):
            raise ValueError("float_val is out of range of UPPER_LIMIT or LOWER_LIMIT")

            #Sets float_val to the bounds
            if (float_val > UPPER_LIMIT):
                float_val = UPPER_LIMIT
            else:
                float_val = LOWER_LIMIT

        if float_val < 0.0:
            fixed_prime = int(round(abs(float_val) * 2**FRAC_BIT_WIDTH))
            fixed_val = fixed_prime | SIGN_BYTE #Sets the MSb to one indicatng a negative val
        else:
            fixed_val = int(round(abs(float_val) * 2**FRAC_BIT_WIDTH))

        return fixed_val


    #Converts fixed point(represented as an 8 byte int) based of the scheme described in the wiki.
    def fixed_point_to_float(self, fixed_val):
        #If a value greater than 255 is received then this single byte encoding scheme won't work.
        if fixed_val > 255:
            raise ValueError("fixed_val: higher than 255")

        #The MSb determines if it is positive or negative, hence being greater than 128 tells the sign(pos = 0, neg = 1)
        if fixed_val > 128:
            is_neg = True
        else: 
            is_neg = False

        int_val = (fixed_val & 0b01110000) >> 4
        decimal_val = (fixed_val & 0b00001111) * 2**-4

        float_val = int_val + decimal_val

        if is_neg:
            float_val *= -1

        return float_val




    def run(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(ROS_SLEEP_RATE)
                        
            #read the data
            self.__bytes = self.__hid_device.read(self.__endpoint.bEndpointAddress, 8, timeout=5000)
            self.mbed_recieve_handler(self.__bytes)
        
            self.__hid_device.write(1, self.__data)
        
            rate.sleep()





if __name__ == "__main__":
    mbedInterface = MbedInterface()
    mbedInterface.run()
