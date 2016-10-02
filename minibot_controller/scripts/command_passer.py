#!/usr/bin/env python

import rospy, socket
from geometry_msgs.msg import Twist

SIGNIFICANCE = 2
def LimitSignificance(value):
    value = str(value)
    decimalPoint = value.find(".")
    value = value[0:decimalPoint + SIGNIFICANCE + 1]
    return value


class command_passer(object):
    def __init__(self):
        rospy.init_node("command_passer")
        driveTopic = rospy.get_param("topics/drive_cmds", "cmd_vel")
        self.hostname = rospy.get_param("hostNameOfMinibot","192.168.1.241")#IP address
        self.port = rospy.get_param("portNumberOfMinibot",9999)
        
        rospy.Subscriber(driveTopic, Twist, self.velSub)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def velSub(self, data):
        linear = data.linear.x
        angular = data.angular.z
        servo = 0

        linear = LimitSignificance(str(linear))
        angular = LimitSignificance(str(angular))
        servo = LimitSignificance(str(servo))
        messageToSend = linear+":"+angular+":"+servo
        self.sock.sendto(messageToSend, (self.hostname, self.port))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    commandPasser = command_passer()
    commandPasser.run()