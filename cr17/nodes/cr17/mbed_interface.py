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

This node must be run under su privelages. I've been doing it using sudo su, then the command to run the node.
'''

##Send to MBed Package Description(8 bytes)
#0 CMD_Vel: Z-Angular
#1 CMD_Vel: X-inear
#2 Scoop  : Arm-Angle-MSB
#3 Scoop  : Arm-ANgle-LSB
#4 Unused : Scoop-Angle-MSB
#5 Unused : Scooop-Angle-LSB
#6 Unused :
#7 Unused :

##Receive from MBed Package Description(8 bytes)
#0 wheelData :  FrntLVel
#1 wheelData :  FrntRVel
#2 wheelData :  BackLVel
#3 wheelData :  BackRVel
#4 Unused :
#5 Unused :
#6 Unused :
#7 Unused :

DATA_ARRAY_SIZE = 8

ROS_SLEEP_RATE = 10

MBED_VENDOR_ID  = 0x1234
MBED_PRODUCT_ID = 0x0006


###Single Byte Implementation Constants##
# 4 bits for fraction + 3 bits for integer + 1 bit for sign = 8 bits
FRAC_BIT_WIDTH = 4

SIGN_BYTE = 0b10000000      #Used to designae the sign of the float(the MSb decides whether it is pos or neg)

#Float upper and lowewr limits
SINGLE_BYTE_FLOAT_UPPER_LIMIT = 7.9
SINGLE_BYTE_FLOAT_LOWER_LIMIT = -7.9

#Signle Byte upper and lower limts
BYTE_UPPER_LIMIT = 255
BYTE_LOWER_LIMIT = 0

#Used with a bitwise AND operation to seperate the bits designating the integer portion of the float and the bits representing the fraction
INT_BITS_AND     = 0b01110000
FRAC_BITS_AND    = 0b00001111

#Used to make room/take away for the fractional bits
INT_BITS_SHIFT = 4

###Two Byte Implementation Constants###

#TWO Byte float upper/lower limits
TWO_BYTE_FLOAT_UPPER_LIMIT = 511.9      # 2^13 * 2^FRAC_BIT_WIDTH
TWO_BYTE_FLOAT_LOWER_LIMIT = 0.0

#The upper limts of the MSB and the LSB
MSB_UPPER_LIMIT     = 255
LSB_UPPER_LIMIT     = 31

#The lower limts of the MSB and the LSB
MSB_LOWER_LIMIT     = 0
LSB_LOWER_LIMIT     = 0

#USed to shift MSB around
MSB_SHIFT           = 5

#Used to isolate the LSB
LSB_AND             = 0b0000000011111

######################################
#Topic Variables
######################################
DRIVE_TOPIC = "/cmd_vel"
SCOOP_TOPIC = "/cmd_scoop"
WHEEL_SPEED_TOPIC = "/wheel_speed"
#SCOOP_OUT = "scoop_out" topic to eventually send scoop values over

######################################
# Needed Functions
######################################

#Converts two ints used into a floating point value. The implementation is described in the wiki.
def fixed_to_float_2(int_MSB, int_LSB):

    #Makes sure both fixed val inputs are ints
    if ((type(int_MSB) != int) | (type(int_LSB) != int)):
        raise TypeError

    #Makes sure the fixed value uses only 13 bits, and sticks to the correct scheme
    if ((int_MSB > MSB_UPPER_LIMIT) | ( int_LSB > LSB_UPPER_LIMIT)):
        raise ValueError("One of the two bytes is too large a value.")

    if ((int_MSB < MSB_LOWER_LIMIT) | (int_LSB < LSB_LOWER_LIMIT)):
        raise ValueError("One of the two bytes is too small a value.")

    fixed_val = (int_MSB << MSB_SHIFT) | int_LSB #Shifts int_MSB by MSB_RIGHT_SHIFT in order to make room for the LSB portion

    float_val = fixed_val* 2.0**-FRAC_BIT_WIDTH

    return float_val

#Converts float to fixed point(represented in 2 bytes) based of the scheme described in the wiki.
def float_to_fixed_2(float_val):

    if(type(float_val) != float):
        raise TypeError

    if (float_val > TWO_BYTE_FLOAT_UPPER_LIMIT) | (float_val < TWO_BYTE_FLOAT_LOWER_LIMIT):
        raise ValueError("Out of range for unsigned two byte implementation: > 511.9 or < 0.0")

    fixed_val = fixed_val = int(round(float_val * 2**FRAC_BIT_WIDTH))
    int_MSB = fixed_val >> MSB_SHIFT
    int_LSB = fixed_val & LSB_AND

    return int_MSB, int_LSB


#Converts float to fixed point(represented as an 8 byte int) based of the scheme described in the wiki.
def float_to_fixed(float_val):
    #Test to make sure input type is float
    if(type(float_val) != float):
        raise TypeError

    #Raises an error if values are out of the limts of what this unibyte encoding can handle
    if (float_val > SINGLE_BYTE_FLOAT_UPPER_LIMIT) | (float_val < SINGLE_BYTE_FLOAT_LOWER_LIMIT):
        raise ValueError("Out of range for signed single byte implementation: > 7.9 or < -7.9")

    if float_val < 0.0:
        fixed_prime = int(round(abs(float_val) * 2**FRAC_BIT_WIDTH))
        fixed_val = fixed_prime | SIGN_BYTE #Sets the MSb to one indicatng a negative val
    else:
        fixed_val = int(round(abs(float_val) * 2**FRAC_BIT_WIDTH))

    return fixed_val


#Converts fixed point(represented as an 8 byte int) based of the scheme described in the wiki.
def fixed_to_float(fixed_val):

    #Test to make sure input is an int
    if(type(fixed_val) != int):
        raise TypeError

    #If a value greater than 255 is received then this single byte encoding scheme won't work.
    if (fixed_val > BYTE_UPPER_LIMIT) | (fixed_val < BYTE_LOWER_LIMIT):
        raise ValueError("Value is higher/lower than the single byte limits(0 or 255)")

    #The MSb determines if it is positive or negative, hence being greater than 128 tells the sign(pos = 0, neg = 1)
    if fixed_val > 128:
        is_neg = True
    else:
        is_neg = False

    int_val = (fixed_val & INT_BITS_AND) >> INT_BITS_SHIFT
    decimal_val = (fixed_val & FRAC_BITS_AND) * 2.0**-4

    float_val = int_val + decimal_val

    if is_neg:
        float_val = -float_val

    return float_val


class MbedInterface(object):
    def __init__(self):
        rospy.init_node('mbed_interface')

        #These will be updated as new data is received for it.
        self.__cmd_vel = Twist()
        self.__cmd_scoop = scoopControl()
        self.__wheel_data = wheelData()

        #Raw data list(byte array?) to/from the Mbed
        self.__data = [0x0] * DATA_ARRAY_SIZE  #Initializes bytes to be set to zero
        self.__bytes = None

        ######################################
        # Setup ROS Subscribers for this node
        ######################################
        rospy.Subscriber(DRIVE_TOPIC, Twist, self.cmd_vel_callback)
        rospy.Subscriber(SCOOP_TOPIC, scoopControl, self.cmd_scoop_callback)


        ######################################
        # Setup ROS Publishers for this node
        ######################################
        self.wheel_speed_pub = rospy.Publisher(WHEEL_SPEED_TOPIC, wheelData, queue_size = 10)

        #This sets up the code for the USB. I'm not 100% what it all does, but it works and is entirely based off the example.
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
        #print "Received Twist from ", DRIVE_TOPIC
        self.__data[0] = float_to_fixed(twist_msg.angular.z)
        self.__data[1] = float_to_fixed(twist_msg.linear.x)

    #Add new scoop cmd to USB message(arm velocity/scoop velocity)
    def cmd_scoop_callback(self, scoop_msg):
        #print "Received scoopControl from ", SCOOP_TOPIC
        self.__data[2], self.__data[3] = float_to_fixed_2(scoop_msg.armAngle)
        self.__data[4], self.__data[5] = float_to_fixed_2(scoop_msg.scoopAngle)

    #Sends data recieved from Mbed over a topic(wheelData)
    def mbed_recieve_handler(self, data):

        #publish out all the received MBED data

        #Sets up wheelData message
        self.__wheel_data.frontLeftVel  = fixed_to_float(data[0])
        self.__wheel_data.frontRightVel = fixed_to_float(data[1])
        self.__wheel_data.backLeftVel   = fixed_to_float(data[2])
        self.__wheel_data.backRightVel  = fixed_to_float(data[3])

        #print self.__wheel_data.frontRightVel
        #print self.__wheel_data.frontLeftVel

        #publishes wheelData message to topic
        self.wheel_speed_pub.publish(self.__wheel_data)

		#TODO: Send Values Scoop Position Value to TF Tree
        #print fixed_to_float_2(data[4], data[5])
        #print fixed_to_float_2(data[6], data[7])


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
    print mbedInterface.run()
