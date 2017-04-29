#!/usr/bin/env python2
import rospy
from cr17.msg import dockerStatus
from cr17.srv import dockerState, dockerStateResponse
from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


POSE_TOPIC = rospy.get_param("topics/localization_pose")
DOCKER_TOPIC = rospy.get_param("topics/docker_status")
DRIVE_TOPIC = rospy.get_param("topics/drive_cmds")

Y_TARGET = rospy.get_param("docker/y_target")
Y_RANGE = rospy.get_param("docker/y_range")
Y_SCALAR = rospy.get_param("docker/y_scalar")
YAW_TARGET = rospy.get_param("docker/yaw_target")
YAW_RANGE = rospy.get_param("docker/yaw_range")
YAW_SCALAR = rospy.get_param("docker/yaw_scalar")


class Docker(object):
    def __init__(self):
        rospy.init_node('docker')

        self.dockerActive = False
        self.pose = Pose()

        self.dockingReverse = False

        #ROS Publishers
        self.drivePub = rospy.Publisher(DRIVE_TOPIC, Twist, queue_size = 10)
        self.dockerStatusPub = rospy.Publisher(DOCKER_TOPIC, dockerStatus, queue_size = 10)

        #ROS Subscribers
        rospy.Subscriber(POSE_TOPIC, PoseStamped, self.pose_sub)

        # ROS Services
        rospy.Service('/dockerState', dockerState, self.set_docker_state)

    def pose_sub(self, data):
        self.pose = data.pose

    def set_docker_state(self, data):
        if (self.dockerActive != data.newDockerState) and (data.newDockerState):
            self.dockingReverse = True
        self.dockerActive = data.newDockerState
        return dockerStateResponse(self.dockerActive)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            currentStatus = dockerStatus()
            currentStatus.dockerActive = self.dockerActive

            if self.dockerActive:
                currentY = self.pose.position.y
                currentOrientation = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]) 
                currentYaw = currentOrientation[2]

                driveTwist = Twist()
                driveTwist.linear.x = (currentY - Y_TARGET) * Y_SCALAR
                driveTwist.angular.z = (currentYaw - YAW_TARGET) * YAW_SCALAR

                currentStatus.dockingComplete = False

                if abs(currentY - Y_TARGET) < Y_RANGE:
                    if abs(currentYaw - YAW_TARGET) < YAW_RANGE:
                        currentStatus.dockingComplete = True
                        driveTwist.linear.x = 0
                        driveTwist.angular.z = 0

                self.drivePub.publish(driveTwist)

            elif self.dockingReverse:
                driveTwist = Twist()
                driveTwist.linear.x = -20
                for each in range(0, 30):
                    self.drivePub.publish(driveTwist)
                    rate.sleep()
                self.dockingReverse = False

            else:
                currentStatus.dockingComplete = False

            self.dockerStatusPub.publish(currentStatus)
            rate.sleep()



if __name__ == "__main__":
	docker = Docker()
	docker.run()
