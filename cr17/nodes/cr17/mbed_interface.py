#!/usr/bin/env python2
import rospy
from geometry_messages.msg import Twist
from cr17.msg import wheelData, scoopControl

'''
Sends low-level commands(scoop, arm, linear velocity, angular velocity) to Mbed over USB.
Also accepts speed values from Mbed for each wheel, and publishes to the wheel_speed topic. 
'''

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

        rospy.Subscriber(DRIVE_TOPIC, Twist,   cmd_vel_callback)
        rospy.Subscriber(SCOOP_TOPIC, scoopControl, cmd_scoop_callback)


        ######################################
        # Setup ROS publishers
        ######################################
        self.wheel_speed_pub = rospy.Publisher(WHEEL_SPEED_TOPIC, Twist, queue_size = 10)


    #Add new cmd_vel to USB message(x-linear/z-angular)
    def cmd_vel_callback(self, data):
        print "Received Twist from ", DRIVE_TOPIC

    #Add new scoop cmd to USB message(arm velocity/scoop velocity)
    def cmd_scoop_callback(self, data):
        print "Received scoopControl from ", SCOOP_TOPIC

    #Sends data recieved from Mbed over a topic(wheelData)
    def mbed_recieve_handler(self, data):
        print "Received data from Mbed."



    def run(self):
        rospy.spin()





if __name__ == "__main__":
	mbedInterface = MbedInterface()
	mbedInterface.run()
