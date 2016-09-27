#!/usr/bin/env python2

import rospy, signal, atexit, math
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool
'''
This module is responsible for sending Twist commands to robot.
Input: Waypoints, Map
Output: Twist commands
'''
######################################
# Global constants
GOAL_FORCE_CONST    = 1.0  # magnitude used when calculating goal force
ANGULAR_SPEED       = 7.0
LINEAR_SPEED        = 7.0
RYAN_CONSTANT       = 7
GOAL_THRESH         = 0.1       # radius around goal that it's okay to stop in 
######################################
# Load global topic names from ros params
######################################
DRIVE_TOPIC = rospy.get_param("topics/drive_cmds_pfield", "cmd_vel_pfield")
GOAL_TOPIC = rospy.get_param("topics/navigation_goals", "nav_goal")
ROBOPOSE_TOPIC = rospy.get_param("topics/particleFilter_pose_out", "beacon_localization_pose")
BEACON_LOST_TOPIC = rospy.get_param("topics/beacon_lost", "beacon_lost")
REACHED_GOAL_TOPIC = rospy.get_param("topics/reached_goal", "reached_goal")
OBSTACLES_TOPIC = rospy.get_param("topics/obstacles","/obstacle_centroids")

def at_goal(robot_pose, goal):
    '''
    Given a robot_pose and a goal coordinate, this node determines if robot is at the goal 
    '''
    # calc distance 
    dist = math.sqrt((goal.x - robot_pose.position.x)**2 + (goal.y - robot_pose.position.y)**2)
    at_goal = True if dist <= GOAL_THRESH else False
    return at_goal

def calc_goal_force(nav_goal, robot_pose):
    '''
    given a goal point and a robot pose, calculate and return x and y components of goal force
    '''
    FIELD_SPREAD = 10.0 # radius around goal where pfield is scaled
    ALPHA = 1.0
    # get distance between goal and robot
    dist = math.sqrt((nav_goal.x - robot_pose.position.x)**2 + (nav_goal.y - robot_pose.position.y)**2)
    # get angle to goal
    angle_to_goal = math.atan2(nav_goal.y - robot_pose.position.y, nav_goal.x - robot_pose.position.x)
    # get force angle
    force_angle = wrap_angle(angle_to_goal)
    # math the components
    if dist < GOAL_THRESH:
        d_x = 0
        d_y = 0
    elif (GOAL_THRESH <= dist) and (dist <= FIELD_SPREAD + GOAL_THRESH):
        d_x = ALPHA * (dist - GOAL_THRESH) * math.cos(force_angle)
        d_y = ALPHA * (dist - GOAL_THRESH) * math.sin(force_angle)
    else: #dist > (FIELD_SPREAD + GOAL_THRESH)
        d_x = ALPHA * FIELD_SPREAD * math.cos(force_angle)
        d_y = ALPHA * FIELD_SPREAD * math.sin(force_angle)

    return [d_x, d_y]


def wrap_angle(angle):
    #This function will take any angle and wrap it into the range [-pi, pi]
    while angle >= math.pi:
        angle = angle - 2 * math.pi
        
    while angle <= -math.pi:
        angle = angle + 2 * math.pi
    return angle

def calc_repulsive_force(obstacles, robot_pose):
    '''
    Given a list of obstacles and robot pose, calculate repulsive force
    '''
    rep_force = [0, 0]
    OBST_THRESH = 0.01
    FIELD_SPREAD = 0.5
    LARGE_NUMBER = 99
    BETA = 1.0          # scale factor
    for obstacle in obstacles:
        # calculate distance between robot and obstacle
        dist = math.sqrt((obstacle[0] - robot_pose.position.x)**2 + (obstacle[1] - robot_pose.position.y)**2)
        # calculate angle between robot and obstacle
        angle_to_obs = math.atan2(obstacle[1] - robot_pose.position.y, obstacle[0] - robot_pose.position.x)
        if dist < OBST_THRESH:
            # too close
            d_x = LARGE_NUMBER if math.cos(angle_to_obs) < 0.0 else -LARGE_NUMBER
            d_y = LARGE_NUMBER if math.sin(angle_to_obs) < 0.0 else -LARGE_NUMBER
        elif (OBST_THRESH <= dist) and (dist <= FIELD_SPREAD + OBST_THRESH):
            # field is scaled
            d_x = -BETA * (FIELD_SPREAD + OBST_THRESH - dist) * math.cos(angle_to_obs)
            d_y = -BETA * (FIELD_SPREAD + OBST_THRESH - dist) * math.sin(angle_to_obs)
        else:
            # obstacle is far away, don't care about it
            d_x = 0
            d_y = 0
        rep_force[0] += d_x
        rep_force[1] += d_y
    return rep_force


class PFieldNavigator(object):

    def __init__(self):
        '''
        Potential field navigator constructor.
        '''
        rospy.init_node("pfield_navigator")
        self.robot_pose = Pose()
        self.received_pose = False
        self.current_goal = Point()
        self.beacon_lost = True
        self.centroid_obstacles = None

        self.previousDirection = 1.0 #initalize it to move forward more often
        
        ######################################
        # Setup ROS publishers
        ######################################
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, Twist, queue_size = 10)
        self.reached_goal_pub = rospy.Publisher(REACHED_GOAL_TOPIC, Bool, queue_size = 10)

        ######################################
        # Setup ROS subscibers
        ######################################
        rospy.Subscriber(ROBOPOSE_TOPIC, PoseStamped, self.robot_pose_callback)
        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.nav_goal_callback)
        rospy.Subscriber(BEACON_LOST_TOPIC, Bool, self.beacon_lost_callback)
        rospy.Subscriber(OBSTACLES_TOPIC, PointCloud, self.obstacle_callback)

    def nav_goal_callback(self, data):
        '''
        Callback for navigation goals.
        '''
        #print("Received nav goal: " + str(data))
        self.current_goal = data.pose.position

    def beacon_lost_callback(self, data):
        '''
        Callback for beacon_lost messages
        '''
        self.beacon_lost = data.data

    def robot_pose_callback(self, data):
        '''
        Callback for robot localization pose.
        '''
        self.received_pose = True
        self.robot_pose = self.transform_pose(data.pose)
        
    def obstacle_callback(self, pc):
        self.centroid_obstacles = pc

    def transform_pose(self, pose):
        '''
        Given a global pose, transform to local robot pose
        '''
        # convert quat orientation to eulers
        global_orient = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]) 
        
        roll = global_orient[0]
        pitch = global_orient[1]
        yaw = global_orient[2]

        local_yaw = yaw - math.pi / 2

        robot_orient = quaternion_from_euler(roll, pitch, local_yaw)

        local_pose = pose
        local_pose.orientation.x = robot_orient[0]
        local_pose.orientation.y = robot_orient[1]
        local_pose.orientation.z = robot_orient[2]
        local_pose.orientation.w = robot_orient[3]

        return local_pose

    def run(self):
        '''
        '''
        rate = rospy.Rate(10)
        # hold up for some messages
        rospy.wait_for_message(ROBOPOSE_TOPIC, PoseStamped)
        rospy.wait_for_message(GOAL_TOPIC, Point)
        temp_obstacles = []
        # work hard doing good stuff
        while not rospy.is_shutdown():
        	#New pose and the beacon is identified
            if self.received_pose and not self.beacon_lost:
                self.received_pose = False
                # grab current goal and pose information
                nav_goal = self.current_goal
                robot_pose = self.robot_pose 
                print("==============================")
                print("Navigating...")
                print(" **  Goal: \n" + str(nav_goal))
                print(" ** Position: \n" + str(robot_pose.position))
                # Check to see if at goal
                if at_goal(robot_pose, nav_goal):
                    # Robot has made it to goal.
                    self.reached_goal_pub.publish(Bool(True))
                else:
                    # Robot still going towards goal.
                    self.reached_goal_pub.publish(Bool(False))

                # Calculate goal force
                attr_force = calc_goal_force(nav_goal, robot_pose)
                print("Goal force: " + str(attr_force))
                # Calculate repulsive force
                #repulsive_force = calc_repulsive_force(self.centroid_obstacles, robot_pose)
                # Get final drive vector (goal, obstacle forces)
                # Calculate twist message from drive vector
                drive_cmd = self.drive_from_force(attr_force, robot_pose)
                self.drive_pub.publish(drive_cmd)

            #Beacon lost
            elif self.beacon_lost:
                print("Beacon Lost")
                # the beacon is lost, turn search for beacon
                self.received_pose = False
               	cmd = Twist()
               	cmd.angular.z = 6
               	self.drive_pub.publish(cmd)
            else:
                pass

            rate.sleep()


    def drive_from_force(self, force, robot_pose):
        '''
        Given a force vector, generate Twist message 
        '''
        cmd = Twist()
        max_angle = math.pi
        spin_thresh = math.pi / 4.0 #I added the / 4.0 to see if it helps the back up
        # get force magnitude
        force_mag = math.hypot(force[0], force[1])
        if force_mag == 0: return cmd
        # normalize force
        force[0] = force[0] / float(force_mag)
        force[1] = force[1] / float(force_mag)
        # convert quat orientation to eulers
        robot_orient = euler_from_quaternion([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w]) 
        # Get force angle (in global space)
        force_angle = math.atan2(force[1], force[0])
        # put force angle in robot space
        force_angle = -1 * (force_angle - (math.pi / 2.0))

        #this is for when the force is behind the robot
        signed_lin_vel = LINEAR_SPEED  #this number needs to be changed, we don't want it to move forward quickly while turning, so it should be affected by angular velocity

        if abs(force_angle) > math.radians(90 + self.previousDirection * 20):#previousDirection math makes it more likly to back up again if it was just backing up
        	#pi-|angle| gives you the magnitude of the angle
        	# angle/|angle| will be the sign of the original angle
        	force_angle = (math.pi - abs(force_angle)) * (-1 * (force_angle / abs(force_angle)))
        	signed_lin_vel *= -1.0
        	self.previousDirection = -1.0
        else:
        	self.previousDirection = 1.0

        # get difference to robot's current yaw
        angle_diff = wrap_angle(force_angle - robot_orient[2])
        print("Robot Yaw: " + str(math.degrees(robot_orient[2])))
        print("Force angle: " + str(math.degrees(force_angle)))
        print("Force Magnitude: " + str(force_mag))
        print("Angle diff: " + str(math.degrees(angle_diff)))

        ang_vel = (angle_diff / max_angle) * ANGULAR_SPEED
        lin_vel = 0 if abs(angle_diff) >= spin_thresh else signed_lin_vel

        vel = self._transform_for_ryan(ang_vel, lin_vel)

        print("Ang vel: " + str(vel[0]))
        print("Lin Vel: " + str(vel[1]))
        cmd.angular.z = vel[0]
        cmd.linear.x = vel[1]

        return cmd

    def _transform_for_ryan(self, ang_vel, lin_vel):
        '''
        Given angular velocity and linear velocity, transform for ryan (-7, 7)
        Contact: Ryan Smith, (228) 623 - 9492
        '''
        # Normalize, then multiply by seven
        mag = math.sqrt(ang_vel**2 + lin_vel**2)
        ang_vel /= mag
        lin_vel /= mag
        return [ang_vel * RYAN_CONSTANT, lin_vel * RYAN_CONSTANT]


if __name__ == "__main__":
    navigator = PFieldNavigator()
    navigator.run()
