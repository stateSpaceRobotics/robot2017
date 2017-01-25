#!/usr/bin/env python2

import rospy
import socket
import threading
import json
import subprocess
import roslib.message
from sensor_msgs.msg import Joy


MCAST_GRP = '224.1.1.1'
PORT = 1025
try:
    MY_IP_ADDR = subprocess.check_output(["ifconfig", "wlan0"]).split("inet addr:")[1].split(" ")[0]
except:
    print("wlan0 failed, trying eth0")
    try:
        MY_IP_ADDR = subprocess.check_output(["ifconfig", "eth0"]).split("inet addr:")[1].split(" ")[0]
    except:
        print("eth0 failed, trying eth1")
        MY_IP_ADDR = subprocess.check_output(["ifconfig", "eth1"]).split("inet addr:")[1].split(" ")[0]


class joy_repubber(object):
    def __init__(self):
        rospy.init_node("joy_repubber")
        self.joy_pub = rospy.Publisher("joy", Joy, queue_size = 10)

        print("starting socket")


        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((MCAST_GRP, PORT))
        self.host = MY_IP_ADDR#socket.gethostbyname(socket.gethostname())
        self.sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))#socket.INADDR_ANY)
        self.sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)

        print("opened socket")

    def run(self):
        rate = rospy.Rate(10)
        maxsize = 65535
        print("about to loop")
        while not rospy.is_shutdown():
            try:
                data2, addr = self.sock.recvfrom(maxsize)
                data = Joy()
                data.deserialize(data2)
            except socket.error, e:
                print 'Exception'
                continue
            
            self.joy_pub.publish(data)
            rate.sleep()
            # print("loop")



if __name__ == "__main__":
    ctrlStation = joy_repubber()
    ctrlStation.run()
