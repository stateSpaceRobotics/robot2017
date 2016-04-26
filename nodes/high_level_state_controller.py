#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped, Twist, Pose
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Path


CUR_ARM_ANGLE_TOPIC = rospy.get_param("topics/cur_arm_angle", "current_arm_angle")
DES_ARM_ANGLE_TOPIC = rospy.get_param("topics/des_arm_angle", "desired_arm_angle")
HAND_STATE_TOPIC = rospy.get_param("topics/hand_state", "hand_state")
TWIST_TOPIC = rospy.get_param("topics/drive_cmds", "cmd_vel")
POSE_TOPIC = rospy.get_param("topics/particleFilter_pose_out", "/particle_filter/pose_out")
PATH_TOPIC = rospy.get_param("topics/path", "/obstacle_path")
GOAL_TOPIC = rospy.get_param("topics/navigation_goals", "nav_goal")
JOY_TOPIC = rospy.get_param("topics/joystick", "joy")
LEFT_DRIVE_AXIS = rospy.get_param("joy/left_drive", 1)
RIGHT_DRIVE_AXIS = rospy.get_param("joy/right_drive", 4)
ARM_DOWN_AXIS = rospy.get_param("joy/arm_down", 2)
ARM_UP_AXIS = rospy.get_param("joy/arm_up", 5)
HAND_DOWN_BUTTON = rospy.get_param("joy/hand_down", 4)
HAND_UP_BUTTON = rospy.get_param("joy/hand_up", 5)
WHEEL_SEPARATION = rospy.get_param("wheel_separation", 1)
WHEEL_RADIUS = rospy.get_param("wheel_radius", 0.25)
ARM_ANGLE_UP = rospy.get_param("arm_up_angle", 100)
ARM_ANGLE_DOWN = rospy.get_param("arm_down_angle", 10)

Y_IN_DUMP_RANGE = 0.01
Y_IN_MINING_AREA = 4.5 #10 cm past edge of mining area
Y_IN_DOCKING_AREA = 1.5

TIME_DUMP_SEC = 10

X_POS_DUMP = 0
Y_POS_DUMP = 0

IS_CLOSE_DIST = 0.4


class high_level_state_controller(object):
    def __init__(self):
        rospy.init_node("high_level_state_controller")

        self.drive_pub = rospy.Publisher(TWIST_TOPIC, Twist, queue_size=10)
        self.arm_pub = rospy.Publisher(DES_ARM_ANGLE_TOPIC, Float64, queue_size=10)
        self.hand_pub = rospy.Publisher(HAND_STATE_TOPIC, Bool, queue_size=10)
        self.handState = False
        self.goal_pub = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=10)

        rospy.Subscriber(POSE_TOPIC, PoseStamped, self.pose_sub)
        self.pose = PoseStamped()
        rospy.Subscriber(CUR_ARM_ANGLE_TOPIC, Float64, self.arm_sub)
        self.arm_cur_angle = 45.0
        rospy.Subscriber(PATH_TOPIC, Path, self.path_sub)
        self.pathPoses = []
        self.curPathIndex = 0
        rospy.Subscriber(JOY_TOPIC, Joy, self.joy_sub)
        self.autoState = "INIT"
        self.highState = "AUTO"
        self.lastJoy = None
        self.leftDrive = None
        self.rightDrive = None
        self.armDown = None
        self.armUp = None
        self.handDown = None
        self.handUp = None

        self.dumpTimer = None

    def joy_sub(self, data):
        self.lastJoy = data.header.stamp
        self.leftDrive = data.axes[LEFT_DRIVE_AXIS]
        self.rightDrive = -data.axes[RIGHT_DRIVE_AXIS]
        self.armDown = data.axes[ARM_DOWN_AXIS]
        self.armUp = data.axes[ARM_UP_AXIS]
        self.handDown = data.buttons[HAND_DOWN_BUTTON]
        self.handUp = data.buttons[HAND_UP_BUTTON]

    def arm_sub(self, data):
        self.arm_cur_angle = data

    def pose_sub(self, data):
        self.pose = data.pose

    def path_sub(self, data):
        self.pathPoses = data.poses

    def linAngVelFromSkidSteer(self, left, right):
        x = (right+left)*WHEEL_RADIUS/2
        z = (right-left)*WHEEL_RADIUS/WHEEL_SEPARATION
        return x,z

    def cleanLeftJoy(self, leftIn):
        return leftIn

    def cleanRightJoy(self, rightIn):
        return -rightIn

    def combineCleanArm(self, armDown, armUp):
        return (-1*(armUp-1))+(armDown-1)

    def calcHandState(self, handUp, handDown, handState):
        if(handUp and not handDown):
            handState = True
        elif(handDown and not handUp):
            handState = False
        return handState

    def closeTo(self, poseChecked):
        x_dist = abs(self.pose.position.x - poseChecked.pose.position.x)
        y_dist = abs(self.pose.position.y - poseChecked.pose.position.y)
        euclid_dist = x_dist + y_dist

        return (euclid_dist < IS_CLOSE_DIST)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #in autonomous mode
            if(self.highState == "AUTO"):
                #Act on the autoState
                if(self.autostate == "INIT"):
                    pass
                elif(self.autostate == "F_OBSTACLE_FIELD"):
                    if(self.closeTo(self.pathPoses[self.curPathIndex])):
                        self.curPathIndex += 1
                        if(self.curPathIndex >= len(self.pathPoses)):
                            self.curPathIndex = len(self.pathPoses) - 1
                    self.goal_pub.publish(self.pathPoses[self.curPathIndex])
                elif(self.autostate == "MINING_BEHAVIOR"):
                    #first put down the hand completely
                    #If the arm is at a certain angle in the F_OBSTACLE_FIELD state, we could put the hand down, 
                    #   then have the arm go down, so it can use the arm angle to tell when the hand is ready

                    #keep a counter of the number of the times it has already mined, can create path based on angle
                    #   pulled from an array based on iteration, make sure to reset the mining path to None when leaving this state

                    #once the end of the path has been reached, close the hand and iterate back across path

                    #once the beginning has been reached again
                    mining_complete = True
                elif(self.autostate == "B_OBSTACLE_FIELD"):
                    if(self.closeTo(self.pathPoses[self.curPathIndex])):
                        self.curPathIndex -= 1
                        if(self.curPathIndex < 0 ):
                            self.curPathIndex = 0
                    self.goal_pub.publish(self.pathPoses[self.curPathIndex])
                elif(self.autostate == "DOCKING"):
                    dockPose = PoseStamped()
                    dockPose.pose.position.x = X_POS_DUMP
                    dockPose.pose.position.y = Y_POS_DUMP
                    self.goal_pub.publish(dockPose)
                elif(self.autostate == "DUMPING"):
                    if(self.arm_cur_angle >= ARM_ANGLE_UP):
                        if(self.dumpTimer == None):
                            self.dumpTimer = rospy.Time.now() + rospy.Duration(TIME_DUMP_SEC)
                        elif(self.dumpTimer >= rospy.Time.now() ):
                            dumping_complete = True
                            self.arm_pub(ARM_ANGLE_DOWN) #TODO: possibly change to a middle value
                        else:
                            #possibly do shaking stuff
                            pass
                    else:
                        self.arm_pub(ARM_ANGLE_UP)

            elif(self.highState == "TELE"):
                if(self.lastJoy is not None):
                    cmd_vel = Twist()
                    left = self.cleanLeftJoy(self.leftDrive)
                    right = self.cleanRightJoy(self.rightDrive)
                    cmd_vel.linear.x, cmd_vel.angular.z = self.linAngVelFromSkidSteer(left, right)#TODO: might need to rescale
                    self.drive_pub.publish(cmd_vel)

                    armCommand = self.combineCleanArm(self.armDown, self.armUp)
                    ArmMoveAmount = (ARM_ANGLE_UP - ARM_ANGLE_DOWN)/10
                    if(armCommand > .1):
                        self.arm_pub.publish(self.arm_cur_angle+ArmMoveAmount)
                    elif(armCommand < -0.1):
                        self.arm_pub.publish(self.arm_cur_angle-ArmMoveAmount)
                    else:
                        self.arm_pub.publish(self.arm_cur_angle)

                    self.handState = self.calcHandState(self.handUp, self.handDown, self.handState)
                    self.hand_pub.publish(self.handState)

            #advance the autostate
            if(self.autostate == "INIT"):
                if(True):#change to some actual check
                    self.autostate = "F_OBSTACLE_FIELD"
            elif(self.autostate == "F_OBSTACLE_FIELD"):
                if(self.pose.position.y >= Y_IN_MINING_AREA):
                    self.autostate = "MINING_BEHAVIOR"
            elif(self.autostate == "MINING_BEHAVIOR")
                if(mining_complete or (self.pose.position.y < Y_IN_MINING_AREA - .3)): #the or part is incase we merge teleop and autonomy
                    self.autostate = "B_OBSTACLE_FIELD"
            elif(self.autostate == "B_OBSTACLE_FIELD"):
                if(self.pose.position.y <= Y_IN_DOCKING_AREA):
                    self.autostate = "DOCKING"
            elif(self.autostate == "DOCKING"):
                if(self.pose.position.y < Y_IN_DUMP_RANGE):
                    self.autostate = "DUMPING"
            elif(self.autostate == "DUMPING"):
                if(dumping_complete or (self.pose.position.y >= Y_IN_DUMP_RANGE+0.5)):
                    self.autostate = "F_OBSTACLE_FIELD"
                    self.dumpTimer = None
            else:
                self.autostate = "INIT"


                
            rate.sleep()


if __name__ == "__main__":
    hlsc = high_level_state_controller()
    hlsc.run()