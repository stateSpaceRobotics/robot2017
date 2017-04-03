#!/usr/bin/env python2

import rospy, math

from geometry_msgs.msg import PoseStamped, Twist, Pose
from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Path
from cr17.msg import scoopControl
from cr17.srv import autonomousActive, autonomousActiveResponse


ARM_STATE_TOPIC = rospy.get_param("topics/scoop_state_cmds", "scoop_commands")
POSE_TOPIC = rospy.get_param("topics/localization_pose") #rospy.get_param("topics/filtered_pose", "filtered_pose")
PATH_TOPIC = rospy.get_param("topics/path", "/obstacle_path")
GOAL_TOPIC = rospy.get_param("topics/navigation_goals", "nav_goal") #TODO: might need to be renamed
STATE_TOPIC = rospy.get_param("topics/robot_state", "state")
MINING_DISTANCE = rospy.get_param("distance_to_mine", 1.5)

Y_IN_DUMP_RANGE = 0.4
Y_IN_MINING_AREA = 4.5 #10 cm past edge of mining area
Y_IN_DOCKING_AREA = 1.5

TIME_DUMP_SEC = 10

X_POS_DUMP = 0
Y_POS_DUMP = 0.5

IS_CLOSE_DIST = 0.4
ANGLES_TO_MINE = [0, 15, -15, 30, -30, 45, -45, 60, -60]

######################################################################################################
#TODO: add stuff for getting stuck
######################################################################################################


class high_level_state_controller(object):
    def __init__(self):
        rospy.init_node("high_level_state_controller")

        self.state_pub = rospy.Publisher(STATE_TOPIC, String, queue_size=10)

        self.pose = Pose()
        self.autostate = "INIT"

        self.miningAngleIndex = 0
        self.miningReady = False
        self.miningDone = False
        self.miningTarget = None
        self.dumpTimer = None
        self.autonomyEnabled = True

        self.frontMiningGoal = PoseStamped()
        self.frontMiningGoal.header.frame_id = "map"
        self.frontMiningGoal.pose.position.x = 0
        self.frontMiningGoal.pose.position.y = Y_IN_MINING_AREA+0.4
        self.frontMiningGoal.pose.orientation.x = 0
        self.frontMiningGoal.pose.orientation.y = 0
        self.frontMiningGoal.pose.orientation.z = -0.7071067811865476
        self.frontMiningGoal.pose.orientation.w = -0.7071067811865476

        self.dumpSetupGoal = PoseStamped()
        self.dumpSetupGoal.header.frame_id = "map"
        self.dumpSetupGoal.pose.position.x = 0
        self.dumpSetupGoal.pose.position.y = 1.1
        self.dumpSetupGoal.pose.orientation.x = 0
        self.dumpSetupGoal.pose.orientation.y = 0
        self.dumpSetupGoal.pose.orientation.z = -0.7071067811865476
        self.dumpSetupGoal.pose.orientation.w = 0.7071067811865476

        # ROS Publishers
        self.arm_pub = rospy.Publisher(ARM_STATE_TOPIC, scoopControl, queue_size=10)
        self.goal_pub = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=10) #TODO: change to a Path

        #ROS Subscribers
        rospy.Subscriber(POSE_TOPIC, PoseStamped, self.pose_sub)

        # ROS Services
        self.auto_srv = rospy.Service('autonomousHLSC', autonomousActive, self.set_autonomy)

    def set_autonomy(self, data):
        self.autonomyEnabled = data.autonomousActive
        return autonomousActiveResponse(True)

    def pose_sub(self, data):
        self.pose = data.pose

    def arm_drive_state(self):
        newMsg = scoopControl()
        newMsg.desiredState = scoopControl.state_drivingPosition
        self.arm_pub.publish(newMsg)

    def arm_predig_state(self):
        newMsg = scoopControl()
        newMsg.desiredState = scoopControl.state_preDigging
        self.arm_pub.publish(newMsg)

    def arm_dig_state(self):
        newMsg = scoopControl()
        newMsg.desiredState = scoopControl.state_digging
        self.arm_pub.publish(newMsg)

    def arm_postdig_state(self):
        newMsg = scoopControl()
        newMsg.desiredState = scoopControl.state_postDigging
        self.arm_pub.publish(newMsg)

    def arm_predump_state(self):
        newMsg = scoopControl()
        newMsg.desiredState = scoopControl.state_preDump
        self.arm_pub.publish(newMsg)

    def arm_dump_state(self):
        newMsg = scoopControl()
        newMsg.desiredState = scoopControl.state_dump
        self.arm_pub.publish(newMsg)

    def arm_postdump_state(self):
        newMsg = scoopControl()
        newMsg.desiredState = scoopControl.state_postDump
        self.arm_pub.publish(newMsg)

    def calculate_mining_target(self, angle):
        start_x = self.pose.position.x
        start_y = self.pose.position.y

        angle_rads = math.radians(angle)

        end_x = start_x + ( MINING_DISTANCE * math.sin(angle_rads))
        end_y = start_y + ( MINING_DISTANCE * math.cos(angle_rads))

        if abs(end_x) > 1.2:
            end_x = abs(end_x)/end_x * 1.2  #so it won't run into the wall

        #this needs to include the orientation if we use the nav_stack, and since the orientation is a quaternion it will need conversion
        endPose = PoseStamped()
        endPose.header.frame_id = "map"
        endPose.pose.position.x = end_x
        endPose.pose.position.y = end_y
        endPose.pose.orientation.x = 0
        endPose.pose.orientation.y = 0
        endPose.pose.orientation.z = -0.7071067811865476
        endPose.pose.orientation.w = -0.7071067811865476

        return endPose

    def close_to(self, poseChecked):
        try:
            x_dist = abs(self.pose.position.x - poseChecked.position.x)
            y_dist = abs(self.pose.position.y - poseChecked.position.y)
        except AttributeError:
            x_dist = abs(self.pose.position.x - poseChecked.pose.position.x)
            y_dist = abs(self.pose.position.y - poseChecked.pose.position.y)

        euclid_dist = x_dist + y_dist
        print("In close_to")
        return (euclid_dist < IS_CLOSE_DIST)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #Act on the autoState
            if(self.autostate == "INIT"):
                pass
                ######################################################################################################
                #TODO: add code to orient the robot properly, may just set the arm, not sure yet
                ######################################################################################################


            elif(self.autostate == "F_OBSTACLE_FIELD"):
                self.goal_pub.publish(self.frontMiningGoal)
                self.arm_drive_state()

            elif(self.autostate == "MINING_BEHAVIOR"):
                #first prepare to mine (so the scoop is going down while the robot is slowly driving forward)
                if not self.miningReady:
                    self.miningReady = True
                    self.arm_predig_state() #TODO: consider waiting a number of iterations

                #keep a counter of the number of the times it has already mined, can create path based on angle
                #   pulled from an array based on iteration, make sure to reset the mining path to None when leaving this state
                elif(self.miningReady and not self.miningDone ):
                    self.arm_dig_state()

                    if(self.miningTarget == None):
                        self.miningTarget = self.calculate_mining_target(ANGLES_TO_MINE[self.miningAngleIndex])
                        self.miningAngleIndex += 1
                        if(self.miningAngleIndex >= len(ANGLES_TO_MINE)):
                            self.miningAngleIndex = 0

                    self.goal_pub.publish(self.miningTarget)

                    if(self.close_to(self.miningTarget)):
                        self.miningDone = True
                        self.arm_postdig_state()

                #once the end of the path has been reached, close the hand and iterate back across path
                elif self.miningDone:
                    self.arm_postdig_state()
                    self.goal_pub.publish(self.dumpSetupGoal)

                #once the beginning has been reached again

            ######################################################################################################
            #TODO: add code to turn the robot around, probably removing the mining done section of mining state
            ######################################################################################################

            elif(self.autostate == "B_OBSTACLE_FIELD"):
                self.goal_pub.publish(self.dumpSetupGoal)
                self.arm_drive_state()

            elif(self.autostate == "DOCKING"):
                self.arm_predump_state()
                dockPose = PoseStamped()
                dockPose.header.frame_id = "map"
                dockPose.pose.position.x = X_POS_DUMP
                dockPose.pose.position.y = Y_POS_DUMP
                dockPose.pose.orientation.x = 0
                dockPose.pose.orientation.y = 0
                dockPose.pose.orientation.z = -0.7071067811865476
                dockPose.pose.orientation.w = 0.7071067811865476
                self.goal_pub.publish(dockPose)

            elif(self.autostate == "DUMPING"):
                dumping_complete = False
                if(self.dumpTimer == None):
                    self.dumpTimer = rospy.Time.now() + rospy.Duration(secs = TIME_DUMP_SEC)
                elif(self.dumpTimer >= rospy.Time.now() ):
                    dumping_complete = True
                    self.arm_postdump_state()
                else:
                    self.arm_dump_state()

            ######################################################################################################
            #TODO: add code to turn the robot around
            ######################################################################################################


            #advance the autostate
            if(self.autostate == "INIT"):
                if(True):#change to some actual check
                    self.autostate = "F_OBSTACLE_FIELD"
            elif(self.autostate == "F_OBSTACLE_FIELD"):
                if(self.pose.position.y >= Y_IN_MINING_AREA):
                    self.autostate = "MINING_BEHAVIOR"
            elif(self.autostate == "MINING_BEHAVIOR"):
                if( not self.autonomyEnabled or (self.pose.position.y < Y_IN_MINING_AREA - .3) ):
                    self.autostate = "B_OBSTACLE_FIELD"
                    self.miningTarget = None
                    self.miningDone = False
            elif(self.autostate == "B_OBSTACLE_FIELD"):
                if(self.pose.position.y <= Y_IN_DOCKING_AREA):
                    self.autostate = "DOCKING"
            elif(self.autostate == "DOCKING"):
                if(self.pose.position.y < Y_IN_DUMP_RANGE):
                    self.autostate = "DUMPING"
            elif(self.autostate == "DUMPING"):
                if(dumping_complete or (self.pose.position.y >= Y_IN_DUMP_RANGE)):
                    self.autostate = "F_OBSTACLE_FIELD"
                    self.dumpTimer = None
            else:
                self.autostate = "INIT"

            self.state_pub.publish(self.autostate)
            rate.sleep()


if __name__ == "__main__":
    hlsc = high_level_state_controller()
    hlsc.run()
