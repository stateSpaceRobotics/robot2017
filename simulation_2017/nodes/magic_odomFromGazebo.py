#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, PoseArray, PoseStamped

MODELPREFIX = rospy.get_param("magic_modelNamePrefix", "r")

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
                name = data.name[i]
                pose.pose = data.pose[i]
                twist = data.twist[i]
                odom = Odometry()
                odom.pose.pose = pose
                odom.twist.twist = twist

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
            rate.sleep()

if __name__ == "__main__":
    navigator = PFieldNavigator()
    navigator.run()
