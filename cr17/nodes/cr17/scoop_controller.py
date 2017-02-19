#!/usr/bin/env python2
import rospy
from std_msgs.msg import Int16
from cr17.msg import scoopControl
from cr17.srv import autonomousActive

ARM_STATE_TOPIC = rospy.get_param("topics/scoop_state_cmds", "scoop_commands")

ARM_GOAL_TOPIC = rospy.get_param("topics/scoop_cmds", "cmd_sccop")


class ScoopController(object):
    def __init__(self):
        rospy.init_node('scoop_controller')

        # Services
        self.autonomy = rospy.Service('autonomousScoop', autonomousActive, self.set_autonomy)

        # Publishers
        self.scoop_pub = rospy.Publisher(ARM_GOAL_TOPIC, Int16, queue_size=10)

        # Subscribers
        self.arm_sub = rospy.Subscriber(ARM_STATE_TOPIC, scoopControl, queue_size=10)

    def run(self):
        rospy.spin()

    def set_autonomy(self, req):
        self.autonomy = req.autonomousActive
        return True

if __name__ == "__main__":
    scoopController = ScoopController()
    scoopController.run()
