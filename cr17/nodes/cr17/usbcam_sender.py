#!/usr/bin/env python2

import rospy
import socket
import threading
import json
import subprocess
import roslib.message
import StringIO, struct
import roslib
from rospy_message_converter import message_converter, json_message_converter
from sensor_msgs.msg import CompressedImage, CameraInfo

#PICAM_IMAGE_TOPIC = "rosberrypi_cam/image_raw"
PICAM_IMAGE_TOPIC = "/usb_cam/image_raw/compressed"
PICAM_CAMERA_INFO_TOPIC = "/usb_cam/camera_info"

MCAST_GRP = '224.1.1.1'
IMAGEPORT = 1026
INFOPORT = 1036

class usbcam_sender(object):
    def __init__(self):
        rospy.init_node("usbcam_sender")
        
        rospy.Subscriber(PICAM_IMAGE_TOPIC, CompressedImage, self.image_sub)
        rospy.Subscriber(PICAM_CAMERA_INFO_TOPIC, CameraInfo, self.camInfo_sub)
        print("in init")
        self.message = CompressedImage()
        self.newImage = False
        self.newInfo = False

        self.socketSetup = False
        self.socket_setup()


    def socket_setup(self):
        if self.socketSetup:
            return 0
        try:
            MY_IP_ADDR = subprocess.check_output(["ifconfig", "wlan0"]).split("inet addr:")[1].split(" ")[0]
        except:
            print("wlan0 failed, trying eth1")
            try:
                MY_IP_ADDR = subprocess.check_output(["ifconfig", "eth1"]).split("inet addr:")[1].split(" ")[0]
            except:
                print("eth1 failed, trying eth0")
                try:
                    MY_IP_ADDR = subprocess.check_output(["ifconfig", "eth0"]).split("inet addr:")[1].split(" ")[0]
                except:
                    self.socketSetup = False
                    return -1

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        self.host = MY_IP_ADDR
        self.sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.host))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        self.sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(MCAST_GRP) + socket.inet_aton(self.host))
        self.socketSetup = True
        return 1


    def image_sub(self, data):
        self.message = data
        self.newImage = True

    def camInfo_sub(self, data):
        self.camInfo = data
        self.newInfo = True

    def run(self):
        rate = rospy.Rate(7)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self.socketSetup:
                self.socket_setup()
                if notself.socketSetup:
                    continue
            if self.newImage:
                serializedData = StringIO.StringIO()
                self.message.serialize(serializedData)
                self.sock.sendto(serializedData.getvalue(), (MCAST_GRP, int(IMAGEPORT)))
            if self.newInfo:
                serializedData = StringIO.StringIO()
                self.camInfo.serialize(serializedData)
                self.sock.sendto(serializedData.getvalue(), (MCAST_GRP, int(INFOPORT)))

if __name__ == "__main__":
    piCamTx = usbcam_sender()
    piCamTx.run()
