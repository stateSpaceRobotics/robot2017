#!/usr/bin/env python2

import rospy
import socket
import threading
import json
import subprocess
import roslib.message
from rospy_message_converter import message_converter, json_message_converter
from sensor_msgs.msg import CompressedImage, CameraInfo

PINOIR_IMAGE_TOPIC = "usb_cam/image_raw/compressed"
PINOIR_CAMERA_INFO_TOPIC = "usb_cam/camera_info"

MCAST_GRP = '224.1.1.1'
IMAGEPORT = 1026
INFOPORT = 1036

try:
    MY_IP_ADDR = subprocess.check_output(["ifconfig", "wlan0"]).split("inet addr:")[1].split(" ")[0]
except:
    print("wlan0 failed, trying eth0")
    try:
        MY_IP_ADDR = subprocess.check_output(["ifconfig", "eth0"]).split("inet addr:")[1].split(" ")[0]
    except:
        print("eth0 failed, trying eth1")
        MY_IP_ADDR = subprocess.check_output(["ifconfig", "eth1"]).split("inet addr:")[1].split(" ")[0]


class usbcam_receiver(object):
    def __init__(self):
        rospy.init_node("usbcam_receiver")
        self.joy_pub = rospy.Publisher(PINOIR_IMAGE_TOPIC, CompressedImage, queue_size = 10)
        self.camInfo_pub = rospy.Publisher(PINOIR_CAMERA_INFO_TOPIC, CameraInfo, queue_size = 10)
        print("starting socket")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((MCAST_GRP, IMAGEPORT))
        self.host = MY_IP_ADDR
        self.sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))
        self.sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        self.sock.setblocking(False)

        print("opened socket")
        

        self.sockInfo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sockInfo.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 3)
        self.sockInfo.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sockInfo.bind((MCAST_GRP, INFOPORT))
        self.sockInfo.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))
        self.sockInfo.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))
        self.sockInfo.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        self.sockInfo.setblocking(False)


    def run(self):
        rate = rospy.Rate(50)
        #print("here1")
        maxsize = 65535
        print("about to loop")
        while not rospy.is_shutdown():
            #print("here2")
            try:
                data2, addr = self.sock.recvfrom(maxsize)
                data = CompressedImage()
                data.deserialize(data2)
            except socket.error, e:
                #print 'Exception'
                continue
            
            try:
                data2, addr = self.sockInfo.recvfrom(maxsize)
                
                dataInfo = CameraInfo()
                dataInfo.deserialize(data2)
            except socket.error, e:
                #print 'Exception'
                continue
            
            self.joy_pub.publish(data)
            self.camInfo_pub.publish(dataInfo)
            rate.sleep()


if __name__ == "__main__":
    piCamRx = usbcam_receiver()
    piCamRx.run()
