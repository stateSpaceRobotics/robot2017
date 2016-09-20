#!/usr/bin/env python

import rospy
import socket
import threading
import json
import subprocess
import roslib.message
from sensor_msgs.msg import Joy
from rospy_message_converter import message_converter, json_message_converter
from sensor_msgs.msg import Joy


MCAST_GRP = '224.1.1.1'
PORT = 1025
try:
    MY_IP_ADDR = subprocess.check_output(["ifconfig", "eth1"]).split("inet addr:")[1].split(" ")[0]
except:
    print("wlan0 failed, trying eth0")
    try:
        MY_IP_ADDR = subprocess.check_output(["ifconfig", "wlan0"]).split("inet addr:")[1].split(" ")[0]
    except:
        print("eth0 failed, trying eth1")
        MY_IP_ADDR = subprocess.check_output(["ifconfig", "eth0"]).split("inet addr:")[1].split(" ")[0]

class control_station(object):
    def __init__(self):
        rospy.init_node("control_station")
        # self.joy_pub = rospy.Publisher("joy_out", Joy, queue_size=10)
        rospy.Subscriber("joy", Joy, self.joy_sub)
        self.joy = Joy()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        self.host = MY_IP_ADDR#socket.gethostbyname(socket.gethostname())
        self.sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        self.sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))

    def joy_sub(self, data):
        self.joy = data
        # self.joy_pub.publish(data)

    def run(self):
        

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # self.joy_pub.publish(self.joy)
            baseMsg = message_converter.convert_ros_message_to_dictionary(self.joy)
            self.sock.sendto(json.dumps(baseMsg), (MCAST_GRP, int(PORT)))
            rate.sleep()

            # rospy.spin()


if __name__ == "__main__":
    ctrlStation = control_station()
    ctrlStation.run()
