#!/usr/bin/env python

import rospy, math

from geometry_msgs.msg import PoseStamped, Twist, Pose
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Path, Odometry


CUR_ARM_ANGLE_TOPIC = rospy.get_param("topics/cur_arm_angle", "current_arm_angle")
DES_ARM_ANGLE_TOPIC = rospy.get_param("topics/des_arm_angle", "desired_arm_angle")
HAND_STATE_TOPIC = rospy.get_param("topics/hand_state", "hand_state")
TWIST_TOPIC = rospy.get_param("topics/drive_cmds", "cmd_vel")
DRIVE_PFIELD_TOPIC = rospy.get_param("topics/drive_cmds_pfield", "cmd_vel_pfield")
POSE_TOPIC = rospy.get_param("topics/filtered_pose", "filtered_pose")#"/particle_filter/pose_out")
PATH_TOPIC = rospy.get_param("topics/path", "/obstacle_path")
GOAL_TOPIC = rospy.get_param("topics/navigation_goals", "nav_goal")
JOY_TOPIC = rospy.get_param("topics/joystick", "joy")
JOY_OUT_TOPIC = rospy.get_param("topics/joystick_mbed", "joy_mbed")
LEFT_DRIVE_AXIS = rospy.get_param("joy/left_drive", 1)
RIGHT_DRIVE_AXIS = rospy.get_param("joy/right_drive", 4)
ARM_DOWN_AXIS = rospy.get_param("joy/arm_down", 2)
ARM_UP_AXIS = rospy.get_param("joy/arm_up", 5)
HAND_DOWN_BUTTON = rospy.get_param("joy/hand_down", 4)
HAND_UP_BUTTON = rospy.get_param("joy/hand_up", 5)
TELEOP_BUTTON = rospy.get_param("joy/teleop", 8)
WHEEL_SEPARATION = rospy.get_param("wheel_separation", 1)
WHEEL_RADIUS = rospy.get_param("wheel_radius", 0.25)
ARM_ANGLE_UP = rospy.get_param("arm_up_angle", 100)
ARM_ANGLE_DOWN = rospy.get_param("arm_down_angle", 10)
MINING_DISTANCE = rospy.get_param("distance_to_mine", 1.5)

Y_IN_DUMP_RANGE = 0.01
Y_IN_MINING_AREA = 4.5 #10 cm past edge of mining area
Y_IN_DOCKING_AREA = 1.5

TIME_DUMP_SEC = 10

X_POS_DUMP = 0
Y_POS_DUMP = 0

IS_CLOSE_DIST = 0.4
ANGLES_TO_MINE = [0, 30, -30, 60, -60]


class high_level_state_controller(object):
    def __init__(self):
        rospy.init_node("high_level_state_controller")

        self.drive_pub = rospy.Publisher(TWIST_TOPIC, Twist, queue_size=10)
        self.arm_pub = rospy.Publisher(DES_ARM_ANGLE_TOPIC, Float64, queue_size=10)
        self.hand_pub = rospy.Publisher(HAND_STATE_TOPIC, Bool, queue_size=10)
        self.handState = False
        self.goal_pub = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=10)
        self.joy_pub = rospy.Publisher(JOY_OUT_TOPIC, Joy, queue_size=10)

        rospy.Subscriber(DRIVE_PFIELD_TOPIC, Twist, self.pfield_sub)
        self.autoJoy = Joy()
        rospy.Subscriber(POSE_TOPIC, PoseStamped, self.pose_sub)
        self.pose = Pose()
        rospy.Subscriber(CUR_ARM_ANGLE_TOPIC, Float64, self.arm_sub)
        self.arm_cur_angle = 45.0
        rospy.Subscriber(PATH_TOPIC, Path, self.path_sub)
        self.pathPoses = []
        self.curPathIndex = 0
        rospy.Subscriber(JOY_TOPIC, Joy, self.joy_sub)
        self.autostate = "INIT"
        self.highState = "AUTO"
        self.lastJoy = None
        self.leftDrive = None
        self.rightDrive = None
        self.armDown = None
        self.armUp = None
        self.handDown = None
        self.handUp = None
        self.teleopButton_prev = 0

        self.miningAngleIndex = 0
        self.miningPathIndex = 0
        self.miningReady = False
        self.miningDone = False
        self.minePath = None
        self.dumpTimer = None

    def joy_sub(self, data):
        self.lastJoy = data.header.stamp
        self.leftDrive = data.axes[LEFT_DRIVE_AXIS]
        self.rightDrive = -data.axes[RIGHT_DRIVE_AXIS]
        self.armDown = data.axes[ARM_DOWN_AXIS]
        self.armUp = data.axes[ARM_UP_AXIS]
        self.handDown = data.buttons[HAND_DOWN_BUTTON]
        self.handUp = data.buttons[HAND_UP_BUTTON]
        if(data.buttons[TELEOP_BUTTON] == 1):
            if(self.teleopButton_prev == 0):
                if(self.highState != "TELE"):
                    self.highState = "TELE"
                else:
                    self.highState = "AUTO"

        self.teleopButton_prev = data.buttons[TELEOP_BUTTON]

        if(self.highState == "TELE"):
            self.joy_pub.publish(data)

    def pfield_sub(self, data):
        x = data.linear.x
        z = data.angular.z
        left = (x - z * wheel_separation / 2) / wheel_radius
        right = (x + z * wheel_separation / 2) * wheel_radius

        self.autoJoy.axes[LEFT_DRIVE_AXIS] = left
        self.autoJoy.axes[RIGHT_DRIVE_AXIS] = right

        if(self.highState == "AUTO"):
            self.joy_pub.publish(self.autoJoy)


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

    def setHandButton(self):
        if(self.handState):
            self.autoJoy.buttons[HAND_DOWN_BUTTON] = True
            self.autoJoy.buttons[HAND_UP_BUTTON] = False
        else:
            self.autoJoy.buttons[HAND_DOWN_BUTTON] = False
            self.autoJoy.buttons[HAND_UP_BUTTON] = True

    def armDownJoy(self):
        self.autoJoy.axes[ARM_DOWN_AXIS] = 1
        self.autoJoy.axes[ARM_UP_AXIS] = -1

    def armUpJoy(self):
        self.autoJoy.axes[ARM_DOWN_AXIS] = -1
        self.autoJoy.axes[ARM_UP_AXIS] = 1

    def armStableJoy(self):
        self.autoJoy.axes[ARM_DOWN_AXIS] = -1
        self.autoJoy.axes[ARM_UP_AXIS] = -1

    def calcMiningPath(self, angle):
        start_x = self.pose.position.x
        start_y = self.pose.position.y

        angle_rads = math.radians(angle)

        end_x = start_x + ( MINING_DISTANCE * math.sin(angle_rads))
        end_y = start_y + ( MINING_DISTANCE * math.cos(angle_rads))

        path = []
        endPose = PoseStamped()
        endPose.pose.position.x = end_x
        endPose.pose.position.y = end_y

        path.append(self.pose)
        path.append(endPose)
        return path

    def closeTo(self, poseChecked):
        try:
            x_dist = abs(self.pose.position.x - poseChecked.position.x)
            y_dist = abs(self.pose.position.y - poseChecked.position.y)
        except AttributeError:
            x_dist = abs(self.pose.position.x - poseChecked.pose.position.x)
            y_dist = abs(self.pose.position.y - poseChecked.pose.position.y)

        euclid_dist = x_dist + y_dist
        print("In closeTo")
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
                    if(len(self.pathPoses) != 0):
                        if(self.closeTo(self.pathPoses[self.curPathIndex])):
                            self.curPathIndex += 1
                            if(self.curPathIndex >= len(self.pathPoses)):
                                self.curPathIndex = len(self.pathPoses) - 1
                        self.goal_pub.publish(self.pathPoses[self.curPathIndex])
                elif(self.autostate == "MINING_BEHAVIOR"):
                    #first put down the hand completely
                    #If the arm is at a certain angle in the F_OBSTACLE_FIELD state, we could put the hand down, 
                    #   then have the arm go down, so it can use the arm angle to tell when the hand is ready
                    if not self.miningReady:
                        self.miningReady = True
                        self.handState = True
                        self.hand_pub.publish(self.handState)
                        self.setHandButton()

                    #keep a counter of the number of the times it has already mined, can create path based on angle
                    #   pulled from an array based on iteration, make sure to reset the mining path to None when leaving this state
                    elif(self.miningReady and not self.miningDone ):
                        if(self.minePath == None):
                            self.minePath = self.calcMiningPath(ANGLES_TO_MINE[self.miningAngleIndex])
                            self.miningAngleIndex += 1
                            if(self.miningAngleIndex >= len(ANGLES_TO_MINE)):
                                self.miningAngleIndex = 0

                        if(self.closeTo(self.minePath[self.miningPathIndex])):
                            self.miningPathIndex += 1
                            if (self.miningPathIndex >= len(self.minePath)):
                                self.minePath = len(self.minePath) - 1
                        self.goal_pub.publish(self.minePath[self.miningPathIndex])

                        if(self.closeTo(self.minePath[len(self.minePath) - 1])):
                            self.miningDone = True

                    #once the end of the path has been reached, close the hand and iterate back across path
                    elif self.miningDone:
                        self.handState = False
                        self.hand_pub(self.handState)
                        self.setHandButton()
                        if (self.closeTo(self.minePath[self.miningPathIndex])):
                            self.miningPathIndex -= 1
                            if (self.miningPathIndex < 0):
                                self.minePath = 0
                        self.goal_pub(self.minePath[self.miningPathIndex])

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
                            self.arm_pub.publish(ARM_ANGLE_DOWN) #TODO: possibly change to a middle value
                            self.armDownJoy()
                        else:
                            #possibly do shaking stuff
                            pass
                    else:
                        self.arm_pub.publish(ARM_ANGLE_UP)
                        self.armUpJoy()

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
                        self.armUpJoy()
                    elif(armCommand < -0.1):
                        self.arm_pub.publish(self.arm_cur_angle-ArmMoveAmount)
                        self.armDownJoy()
                    else:
                        self.arm_pub.publish(self.arm_cur_angle)
                        self.armStableJoy()

                    self.handState = self.calcHandState(self.handUp, self.handDown, self.handState)
                    self.hand_pub.publish(self.handState)

            #advance the autostate
            if(self.autostate == "INIT"):
                if(True):#change to some actual check
                    self.autostate = "F_OBSTACLE_FIELD"
            elif(self.autostate == "F_OBSTACLE_FIELD"):
                if(self.pose.position.y >= Y_IN_MINING_AREA):
                    self.autostate = "MINING_BEHAVIOR"
            elif(self.autostate == "MINING_BEHAVIOR"):
                if(mining_complete or (self.pose.position.y < Y_IN_MINING_AREA - .3)): #the or part is incase we merge teleop and autonomy
                    self.autostate = "B_OBSTACLE_FIELD"
                    self.minePath = None
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


            # print(self.autostate)
            rate.sleep()


if __name__ == "__main__":
    hlsc = high_level_state_controller()
    hlsc.run()