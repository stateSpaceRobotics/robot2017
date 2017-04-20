#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, PoseArray, PoseStamped

MODELPREFIX = rospy.get_param("magic_modelNamePrefix", "robot")

class PFieldNavigator(object):

    def __init__(self):
        '''
        Potential field navigator constructor.
        '''
        rospy.init_node("magic_gazebo_robot_publisher")
        self.pub_dict = {}
        self.pub_dict2 = {}
        self.pub_dict3 = {}
        self.last_poses = {}

        self.br = tf.TransformBroadcaster()
        self.firstRobotPose = None
        self.robotPose = None
        
        ######################################
        # Setup ROS publishers
        #   but nothing automatically publishes
        ######################################
        # self.drive_pub = rospy.Publisher("miniBot_poses", PoseArray, queue_size = 10)

        ######################################
        # Setup ROS subscibers
        ######################################
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_modelstates_callback)

    def gazebo_modelstates_callback(self, data):
        '''
        Callback for model states
        '''
        i = 0
        poses = PoseArray()
        pose = PoseStamped()
        while i < len(data.name):
            if (data.name[i]).startswith(MODELPREFIX):
                # if self.firstRobotPose == None:
                #     self.firstRobotPose = data.pose[i]
                # self.robotPose = data.pose[i]
                name = data.name[i]
                pose.pose = data.pose[i]
                twist = data.twist[i]
                odom = Odometry()
                odom.pose.pose = pose
                odom.twist.twist = twist

                self.br.sendTransform(
                    (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                    (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
                    rospy.Time.now(),
                    name,
                    "odom"
                )
                self.br.sendTransform(
                    (0,0,0),
                    # (0,0,-0.7071067811865476,0.7071067811865476),
                    # (0.7071067811865476, -0.7071067811865475,-4.329780281177466e-17, 4.329780281177467e-17 ),
                    # (4.329780281177467e-17 , -4.329780281177466e-17, -0.7071067811865475, 0.7071067811865476),
                    (0,0,0,1),
                    rospy.Time.now(),
                    "odom",
                    "map"
                )
                self.br.sendTransform(
                    (0,0,0),
                    (0,0,0,1),
                    rospy.Time.now(),
                    "base_link",
                    "robot"
                )

                self.last_poses[name] = pose
                poses.poses = self.last_poses.values()
                poses.poses.remove(pose)
                if name not in self.pub_dict:
                    self.pub_dict[name] = rospy.Publisher("/"+name+"/odom", Odometry, queue_size = 10)
                    self.pub_dict2[name] = rospy.Publisher("/"+name+"/miniBot_poses", PoseArray, queue_size = 10)
                    self.pub_dict3[name] = rospy.Publisher("/"+name+"/standard_pose", PoseStamped, queue_size = 10)

                self.pub_dict[name].publish(odom)
                self.pub_dict2[name].publish(poses)
                self.pub_dict3[name].publish(pose)
            i += 1
        # self.drive_pub.publish(poses)
        

    def run(self):
        '''
        Do nothing, everything done in callbacks
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # if self.firstRobotPose != None and self.robotPose != None:
            #     self.br.sendTransform(
            #         (self.firstRobotPose.position.x, self.firstRobotPose.position.y, 0),
            #         #(0,0,-0.7071067811865476,0.7071067811865476),
            #         #(0.7071067811865476, -0.7071067811865475,-4.329780281177466e-17, 4.329780281177467e-17 ),
            #         #(4.329780281177467e-17 , -4.329780281177466e-17, -0.7071067811865475, 0.7071067811865476),
            #         #(0,0,0,1),
            #         (self.robotPose.orientation.x, self.robotPose.orientation.y, self.robotPose.orientation.z, self.robotPose.orientation.w),
            #         rospy.Time.now(),
            #         "odom",
            #         "map"
            #     )
            rate.sleep()

if __name__ == "__main__":
    navigator = PFieldNavigator()
    navigator.run()
