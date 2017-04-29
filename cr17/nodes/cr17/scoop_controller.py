#!/usr/bin/env python2
import rospy
from cr17.msg import scoopControl
from cr17.srv import autonomousActive, autonomousActiveResponse


class ScoopController(object):
    def __init__(self):
        rospy.init_node('scoop_controller')

        # Topics
        self.arm_state_topic = rospy.get_param('topics/scoop_state_cmds')
        self.arm_goal_topic = rospy.get_param('topics/scoop_cmds')

        # Scooper/Arm Angles
        self.driving_arm_angle = rospy.get_param('scoop_config/driving_arm_angle')
        self.driving_scoop_angle = rospy.get_param('scoop_config/driving_scoop_angle')

        self.pre_dig_arm_angle = rospy.get_param('scoop_config/pre_dig_arm_angle')
        self.pre_dig_scoop_angle = rospy.get_param('scoop_config/pre_dig_scoop_angle')

        self.digging_arm_angle = rospy.get_param('scoop_config/digging_arm_angle')
        self.digging_scoop_angle = rospy.get_param('scoop_config/digging_scoop_angle')

        self.post_dig_arm_angle = rospy.get_param('scoop_config/post_dig_arm_angle')
        self.post_dig_scoop_angle = rospy.get_param('scoop_config/post_dig_scoop_angle')

        self.pre_dump_arm_angle = rospy.get_param('scoop_config/pre_dump_arm_angle')
        self.pre_dump_scoop_angle = rospy.get_param('scoop_config/pre_dump_scoop_angle')

        self.dumping_arm_angle = rospy.get_param('scoop_config/dumping_arm_angle')
        self.dumping_scoop_angle = rospy.get_param('scoop_config/dumping_scoop_angle')

        self.post_dump_arm_angle = rospy.get_param('scoop_config/post_dump_arm_angle')
        self.post_dump_scoop_angle = rospy.get_param('scoop_config/post_dump_scoop_angle')

        # What should be the starting values for these
        self.autonomous = True
        self.scoop_msg = scoopControl()
        self.defined_msgs = []
        self.create_defined_msgs()

        # ROS Subscribers
        self.arm_sub = rospy.Subscriber(self.arm_state_topic, scoopControl, self.update_scoop_msg, queue_size=10)

        # ROS Publishers
        self.scoop_pub = rospy.Publisher(self.arm_goal_topic, scoopControl, queue_size=10)

        # ROS Services
        self.auto_srv = rospy.Service('autonomousScoop', autonomousActive, self.set_autonomy)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.autonomous:
                # Custom Angles
                if self.scoop_msg.desiredState == self.scoop_msg.state_customAngles:
                    new_msg = scoopControl()
                    new_msg.armAngle = self.scoop_msg.armAngle
                    new_msg.scoopAngle = self.scoop_msg.scoopAngle
                    self.scoop_pub.publish(new_msg)
                else:
                    self.scoop_pub.publish(self.defined_msgs[self.scoop_msg.desiredState])
            rate.sleep()

    def set_autonomy(self, req):
        self.autonomous = req.autonomousActive
        return autonomousActiveResponse(True)

    def update_scoop_msg(self, msg):
        self.scoop_msg = msg

    def create_defined_msgs(self):
        for state_msg in range(0, 8):
            new_msg = scoopControl()
            if state_msg == new_msg.state_drivingPosition:
                new_msg.armAngle = self.driving_arm_angle
                new_msg.scoopAngle = self.driving_scoop_angle
            elif state_msg == new_msg.state_preDigging:
                new_msg.armAngle = self.pre_dig_arm_angle
                new_msg.scoopAngle = self.pre_dig_scoop_angle
            elif state_msg == new_msg.state_digging:
                new_msg.armAngle = self.digging_arm_angle
                new_msg.scoopAngle = self.digging_scoop_angle
            elif state_msg == new_msg.state_postDigging:
                new_msg.armAngle = self.post_dig_arm_angle
                new_msg.scoopAngle = self.post_dig_scoop_angle
            elif state_msg == new_msg.state_preDump:
                new_msg.armAngle = self.pre_dump_arm_angle
                new_msg.scoopAngle = self.pre_dump_scoop_angle
            elif state_msg == new_msg.state_dump:
                new_msg.armAngle = self.dumping_arm_angle
                new_msg.scoopAngle = self.dumping_scoop_angle
            elif state_msg == new_msg.state_postDump:
                new_msg.armAngle = self.post_dump_arm_angle
                new_msg.scoopAngle = self.post_dump_scoop_angle
            self.defined_msgs.append(new_msg)


if __name__ == '__main__':
    scoopController = ScoopController()
    scoopController.run()
