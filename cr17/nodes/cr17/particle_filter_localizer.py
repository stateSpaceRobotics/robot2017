#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from cr17.msg import localizationPoint, localizationPoints
import math
import random

#####################
# BEGIN Global Variable Definitions
particles = []        #Our list of particles
number_of_particles = 500 #How many particles we have
total_weight =0 #Total weight of all particles
partial_weight_sum = [] #The partial weight of each particle
command = None # The most recent drive command
last_filter_update_time = None #When was our last filter update?

#Beacon format: [(distance, angle), (distance, angle), (distance, angle) . . .]
kinect_beacon = None  # The relative location of the beacons (from kinect)
lidar_beacon = None   # The relative location of the beacons (from lidar)

beacon_width = None #How far apart the beacons are
left_beacon = None #The location (x,y) of left beacon
right_beacon = None #The location (x,y) of right beacon

# ARUCO STUFF
aruco_pose = None # Our aruco message estimate (from aruco)
aruco_pose_time = None #Last aruco message time?
aruco_location_sd = 0.1
aruco_theta_sd = 0.05

# KINECT STUFF
kinect_epsilon_d = 0.2 #The distance noise for kinect beacon sensing
kinect_epsilon_a = 0.05 #The angle noise for kinect beacon sensing
kinect_random_reading_p = 0.1 #Probability of a random reading for kinect
kinect_missed_reading_p = 0.05 #Probability of no reading when there should be one
kinect_min_fov_angle = -0.375 #Minimum angle (in robot coordinates) of fov
kinect_max_fov_angle = 0.375 #Maximum angle (in robot coordinates) of fov

# LIDAR STUFF
lidar_epsilon_d = 0.1 #The distance noise for lidar beacon sensing
lidar_epsilon_a = 0.05  #The angle noise for lidar beacon sensing
lidar_random_reading_p = 0.05 #Probability of a random reading for lidar
lidar_missed_reading_p = 0.05 #Probability of no reading when there should be one
lidar_min_fov_angle = -2.36 #Minimum angle (in robot coordinates) of fov
lidar_max_fov_angle = 2.36 #Maximum angle (in robot coordinates) of fov

# INITIAL STUFF
initial_pose = (0,0,0) # initial pose of the robot
initial_location_sd = 0.1 # initial sd of x and y coordinates of robot location
initial_theta_sd = 0.05 #initial sd of robot's theta
#END Global Variable Definitions
#####################

##########################
#CLASS DEFINITIONS
#This class keeps track of our info for a single particle
class Particle:
    def __init__(self,x,y,theta,w=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.w = w

    def display(self):
        print ' (',self.x,',',self.y,',',self.theta,')'
##########################

##########################
# BEGIN ROS Topic Callback Functions
##########################

#Kinect beacon location callback function, store as global variable
def kinectBeaconCallback(data):
    global kinect_beacon
    kinect_beacon = data

#lidar beacon location callback function, store as global variable
def lidarBeaconCallback(data):
    global lidar_beacon
    lidar_beacon = data
    
#aruco pose callback function
def arucoPoseCallback(data):
    global aruco_pose
    global aruco_pose_time
    
    aruco_pose_time = rospy.get_rostime()
    
    apx = data.transform.translation.x
    apy = data.transform.translation.y
    
    apq = data.transform.rotation
    
    [r,p,yaw] = euler_from_quaternion(apq)
    
    aruco_pose = [apx,apy,yaw] 
        
#Command callback, store as global variable
def commandCallback(data):
    global command
    command = data
    advance_particles()
    
############################
##### END CALLBACK FUNCTIONS  
############################

############################
##### MOTION MODEL
############################

#Sample from a triangular distribution
def sample_d(b):
    return (math.sqrt(6)/2)*(random.uniform(-b,b) + random.uniform(-b,b))

# p is the particle to update, u is the twist command, delta_t is time since last update
def sample_motion_model(p, u, delta_t):
    lv = 0.3    # 1. Influence of |linear velocity| on velocity noise
    av = 0.3    # 2. Influence of |angular velocity| on velocity noise
    la = 0.3    # 3. Influence of |linear velocity| on angular noise
    aa = 0.3    # 4. Influence of |angular velocity| on angular noise
    lf = 0.3    # 5. Influence of |linear velocity| on final angle noise
    af = 0.3    # 6. Influence of |angular velocity| on final angle noise

    v = u.linear.x + sample_d(lv*math.fabs(u.linear.x) + av*math.fabs(u.angular.z))
    w = u.angular.z + sample_d(la*math.fabs(u.linear.x) + aa*math.fabs(u.angular.z))
    g = sample_d(lf*math.fabs(u.linear.x) + af*math.fabs(u.angular.z))
    
    x = p.x - (v/w)*math.sin(p.theta) + (v/w)*math.sin(p.theta + w*delta_t)
    y = p.y + (v/w)*math.cos(p.theta) - (v/w)*math.cos(p.theta + w*delta_t)
    theta = p.theta + w*delta_t + g*delta_t

    p.x = x
    p.y = y
    p.theta = theta
    
############################
##### SENSOR MODELS STUFF
############################
#Return the value of the normal pdf with mean and sd at value x
def normpdf(x,mean,sd):
    var = float(sd)**2
    denom = (2*math.pi*var)**0.5
    num = math.exp(-(float(x)-float(mean))**2/(2*var))
    return num/denom

#Return the value of the normal cdf with mean and sd at value x
def normcdf(x,mean,sd):
    z = (float(x)-float(mean))/float(sd)
    if z >= 0.0:
        return 0.5 + 0.5 * math.erf(z / math.sqrt(2.0))
    else:
        return 0.5 * math.erfc(-z / math.sqrt(2.0))

#Return probability of mixture of: 
# 1. zero-mean gaussian at x with sd of sd 
# 2. Uniform random with probability ur
# Any reading outside of given range gets assigned value at edge
def sensor_model(x, sd, ur, min_x, max_x):
    
    #How much probability mass is inside of our region of interest
    alpha = 1.0 / (normcdf(max_x, 0.0, sd) - normcdf(min_x, 0.0, sd))
    
    prob_of_gaussian = alpha * normpdf(x, 0.0, sd)
    
    if x > max_x:
        prob_of_gaussian = alpha * normpdf(max_x, 0.0, sd)
    
    prob_of_random = ur / (max_x - min_x)
     
    return prob_of_random + prob_of_gaussian
        
# This helper function converts input angle to interval [-pi to pi]
def wrap_angle(angle):
    while angle >= math.pi:
        angle = angle - 2*math.pi
    while angle <= -math.pi:
        angle = angle + 2*math.pi
    return angle

def beacon_visible(ea, min_fov_angle, max_fov_angle):
    #Should this particle be able to see this beacon?
    # i.e. does the expected angle lie in field-of-view (fov)? 
    max_fov = wrap_angle(max_fov_angle - min_fov_angle)
    if max_fov < 0:
        max_fov +=  2*math.pi
    exp_a = wrap_angle(ea - min_fov_angle)
    if exp_a < 0:
        exp_a +=  2*math.pi
    return exp_a < max_fov    
    
def get_probability_beacon(p):    

    # Determine where the beacons should be for this particle's pose
    # Left Beacon Expected Location
    expected_angle_left = wrap_angle(p.theta - math.atan2( left_beacon[1] - p.y, left_beacon[0] - p.x )-p.theta)
    expected_distance_left = math.hypot( left_beacon[0] - p.x, left_beacon[1] - p.y )
    
    #Right Beacon Expected Location
    expected_angle_right = wrap_angle(p.theta - math.atan2( right_beacon[1] - p.y, right_beacon[0] - p.x )-p.theta)
    expected_distance_right = math.hypot( right_beacon[0] - p.x, right_beacon[1] - p.y )
    
    ################
    # KINECT BEACONS
    ################
    
    kinect_p = 1.0
    num_assigned_detections = 0

    # LEFT KINECT BEACON
    if beacon_visible(expected_angle_left, kinect_min_fov_angle, kinect_max_fov_angle):
        #Should see the left one
        lkp, found_it = get_probability_sensor_beacon(p, left_beacon, expected_angle_left, expected_distance_left, kinect_beacon, kinect_epsilon_a, kinect_epsilon_d, kinect_random_reading_p, kinect_missed_reading_p)
        kinect_p *= lkp
        if found_it: 
            num_assigned_detections += 1

    # RIGHT KINECT BEACON
    if beacon_visible(expected_angle_right, kinect_min_fov_angle, kinect_max_fov_angle):
        #Should see the right one
        rkp, found_it = get_probability_sensor_beacon(p, right_beacon, expected_angle_right, expected_distance_right, kinect_beacon, kinect_epsilon_a, kinect_epsilon_d, kinect_random_reading_p, kinect_missed_reading_p)
        kinect_p *= rkp
        if found_it: 
            num_assigned_detections += 1
         
    # UNASSIGNED KINECT DETECTIONS (COUNT AS RANDOM READING)
    for i in range(len(kinect_beacon) - num_assigned_detections):
        kinect_p *= kinect_random_reading_p
    
    ###############
    # LIDAR BEACONS
    ###############
    
    lidar_p = 1.0
    num_assigned_detections = 0

    # LEFT LIDAR BEACON
    if beacon_visible(expected_angle_left, lidar_min_fov_angle, lidar_max_fov_angle):
        #Should see the left one
        llp, found_it = get_probability_sensor_beacon(p, left_beacon, expected_angle_left, expected_distance_left, lidar_beacon, lidar_epsilon_a, lidar_epsilon_d, lidar_random_reading_p, lidar_missed_reading_p)
        lidar_p *= llp
        if found_it: 
            num_assigned_detections += 1

    # RIGHT LIDAR BEACON
    if beacon_visible(expected_angle_right, lidar_min_fov_angle, lidar_max_fov_angle):
        #Should see the right one
        rlp, found_it = get_probability_sensor_beacon(p, right_beacon, expected_angle_right, expected_distance_right, lidar_beacon, lidar_epsilon_a, lidar_epsilon_d, lidar_random_reading_p, lidar_missed_reading_p)
        lidar_p *= rlp
        if found_it: 
            num_assigned_detections += 1
         
    # UNASSIGNED LIDAR DETECTIONS (COUNT AS RANDOM READING)
    for i in range(len(lidar_beacon) - num_assigned_detections):
        lidar_p *= lidar_random_reading_p

    return kinect_p*lidar_p
 
# p is the particle, b is beacon (x,y), ea is expected angle, ed is expected distance, m is the message, eps_d is the distance epsilon, eps_a is the angle epsilon, rr_p is the probability of random reading, mr_p is the probability of a missed reading when there should be one
#We know that this beacon is visible, or we wouldn't have called this function
def get_probability_sensor_beacon(p, b, ea, ed, m, eps_a, eps_d, rr_p, mr_p):
    #Look for closest thing to what was expected
    #Find x,y coordinates of each detected beacon, compare to beacon_location
    min_d = beacon_width/3.0 #Minimum distance that detected beacon and expected beacon can 
    #be apart and still count as that beacon
    min_det = None
    
    #each m is (distance, angle)
    for det in m:
        det_x = p.x + det[0]*math.cos(p.theta + det[1]) #detected beacon x location
        det_y = p.y + det[0]*math.sin(p.theta + det[1]) #detected beacon y location
        det_d = math.hypot(det_x - b[0], det_y - b[1]) #detected beacon distance from real_beacon
        if det_d < min_d:
            min_d = det_d
            min_det = det
    
    if min_det is None:
        #We couldn't find a matching detection.  So we have the probability we missed this one.
        return mr_p, False
    
    #Get probability of angle reading
    a_diff = wrap_angle(ea - min_det[0])
    p_a = sensor_model(a_diff, eps_a, rr_p, -math.pi, math.pi)
    
    #Get probability of distance reading
    d_diff = ed - min_det[1]
    p_d = sensor_model(d_diff, eps_d, rr_p, 0.0, 8.0)
    
    #Return the product of these two probabilities
    return (1-mr_p) * p_a * p_d, True

#########################
# END SENSOR MODEL
#########################
    
##########################
# BEGIN PARTICLE FILTER CODE
##########################

#Initialize the global list of particles with n particles
#Randomly distributed around our initial pose
def initialize_particles(n):
    global particles
    global partial_weight_sum
    
    for i in range(n):
        i = get_random_initial_particle()
        particles.append(i)
        partial_weight_sum.append(0.0)
	
#Initialize the particle to our initial location and heading, plus noise
def get_random_initial_particle():
    px = initial_pose[0] + random.gauss(0,initial_location_sd)
    py = initial_pose[1] + random.gauss(0,initial_location_sd)
    ptheta = initial_pose[2] + random.gauss(0,initial_theta_sd)
    return Particle(px,py,ptheta)
    
#Advance particles forward in delta_t in time, using motion model
def advance_particles():
    global particles
    global last_filter_update_time
    
    #Determine how long it has been since particles were advanced
    #Assume current command was valid that whole time
    current_time = rospy.get_rostime()
    current_duration = current_time - last_filter_update_time
    last_filter_update_time = current_time

    aruco_duration = current_time - aruco_pose_time
    
    #Check to see if the aruco pose is valid (received in last 0.5 seconds)
    if aruco_pose_time and aruco_duration.to_sec() < 0.5:
        #We can see the aruco pose, which we basically trust, so
        #Re-initialize all of the particles to this pose
        for p in particles:
            p.x = aruco_pose[0] + random.gauss(0,aruco_location_sd)
            p.y = aruco_pose[1] + random.gauss(0,aruco_location_sd)
            p.theta = aruco_pose[2] + random.gauss(0,aruco_theta_sd)
    
    else:
        #The aruco can't help us
        #Advance each particle using motion model
        for p in particles:
            #Use motion model to get new pose
            sample_motion_model_(p, command, current_duration.to_sec())
                              
                               
#Check to see if a particle is on the map
def particle_on_map(p):
    epsilon = 0.05

    if p.x < -map_width/2.0 + epsilon or p.x > map_width/2.0 - epsilon:
        return False
    if p.y < epsilon or p.y > map_height - epsilon:
        return False
    
    return True

#Assign each particle its weight
def get_particle_weights():
    global particles
    global total_weight
    global partial_weight_sum
    
    total_weight = 0
    
    #Cycle through each particle and assign it a weight
    #The weight should be stored in particles[i].w
    for i in range(len(particles)):
        #See if particle is in an obstacle or off the map, if it is, it gets weight of 0.0
        if particle_on_map(particles[i]):
            particles[i].w = get_probability_beacon(particles[i])
            total_weight = particles[i].w + total_weight
        else:
            particles[i].w = 0.0
            
        partial_weight_sum[i] = total_weight
            
#Get the next set of particles
#By resampling current ones according to weights, with replacement
def resample_particles():
    global particles
    
    new_particles = []

    for i in range(len(particles)):
        r = random.uniform(0, total_weight)
        j = 0
        while partial_weight_sum[j] < r:
            j += 1        
            if j == len(partial_weight_sum):
                j -= 1
                break
        
        p = Particle(particles[j-1].x, particles[j-1].y, particles[j-1].theta)
        new_particles.append(p)
    
    particles = new_particles

# Compute the average pose estimate of the particles and return it
def compute_filtered_pose():
    filter_pose_x = 0
    filter_pose_y = 0
    filter_pose_theta_x = 0
    filter_pose_theta_y = 0
    
    for p in particles:
        filter_pose_x += p.x
        filter_pose_y += p.y
        filter_pose_theta_x += math.cos(p.theta)
        filter_pose_theta_y += math.sin(p.theta)

    #Pose to output
    filtered_pose = Pose()

    #POSITION    
    # X and Y are simple averages
    filtered_pose.position.x = filter_pose_x / len(particles)
    filtered_pose.position.y = filter_pose_y / len(particles)
    
    #ORIENTATION
    # To get average theta, use vector math
    fpt = math.atan2(filter_pose_theta_y, filter_pose_theta_x)   
    
    #Convert to quaternion
    [qx, qy, qz, qw] = quaternion_from_euler([0.0, 0.0, fpt])
    
    #Save in pose
    filtered_pose.orientation.x = qx
    filtered_pose.orientation.y = qy
    filtered_pose.orientation.z = qz
    filtered_pose.orientation.w = qw
        
    return filtered_pose
    
#Update all the particles, done once per iteration
def update_particles(iteration):
    # 1. Advance physics
    advance_particles()

    # 2. Get weights from sensor model
    get_particle_weights()

    # 3. Resample according to weights
    resample_particles()
    
#Main loop
if __name__ == '__main__':

    #Initialize the ros node
    rospy.init_node('particle_filter_localizer', anonymous=True) 
    
    #Declare needed global variables
    global beacon_width
    global left_beacon
    global right_beacon
    global kinect_epsilon_d 
    global kinect_epsilon_a 
    global kinect_random_reading_p
    global kinect_missed_reading_p
    global kinect_min_fov_angle
    global kinect_max_fov_angle
    global lidar_epsilon_d
    global lidar_epsilon_a  
    global lidar_random_reading_p
    global lidar_missed_reading_p
    global map_width
    global map_height
    global initial_pose
    global initial_location_sd
    global initial_theta_sd
    global aruco_location_sd 
    global aruco_theta_sd 
    global number_of_particles
    
    #Get necessary parameters from parameter server
    beacon_width = rospy.get_param('beacon_localization/post_distance', beacon_width)
    kinect_epsilon_d = rospy.get_param('particle_filter/kinect_epsilon_d', kinect_epsilon_d)
    kinect_epsilon_a = rospy.get_param('particle_filter/kinect_epsilon_a', kinect_epsilon_a)
    kinect_random_reading_p = rospy.get_param('particle_filter/kinect_random_reading_p', kinect_random_reading_p)
    kinect_missed_reading_p = rospy.get_param('particle_filter/kinect_missed_reading_p', kinect_missed_reading_p)
    kinect_min_fov_angle = rospy.get_param('particle_filter/kinect_min_fov_angle', kinect_min_fov_angle)
    kinect_max_fov_angle = rospy.get_param('particle_filter/kinect_max_fov_angle', kinect_max_fov_angle)
    lidar_epsilon_d = rospy.get_param('particle_filter/lidar_epsilon_d', lidar_epsilon_d)
    lidar_epsilon_a = rospy.get_param('particle_filter/lidar_epsilon_a', lidar_epsilon_a)
    lidar_random_reading_p = rospy.get_param('particle_filter/lidar_random_reading_p', lidar_random_reading_p)
    lidar_missed_reading_p = rospy.get_param('particle_filter/lidar_missed_reading_p', lidar_missed_reading_p)
    lidar_min_fov_angle = rospy.get_param('particle_filter/lidar_min_fov_angle', lidar_min_fov_angle)
    lidar_max_fov_angle = rospy.get_param('particle_filter/lidar_max_fov_angle', lidar_max_fov_angle)
    
    map_width = rospy.get_param('map_width', map_width)
    map_height = rospy.get_param('map_height', map_height)
    initial_pose = rospy.get_param('initial_pose', (0,0,0))
    
    initial_location_sd = rospy.get_param('particle_filter/initial_location_sd', initial_location_sd)
    initial_theta_sd = rospy.get_param('particle_filter/initial_theta_sd', initial_theta_sd)
    aruco_location_sd = rospy.get_param('particle_filter/aruco_location_sd', aruco_location_sd)
    aruco_theta_sd = rospy.get_param('particle_filter/aruco_theta_sd', aruco_theta_sd)
    number_of_particles = rospy.get_param('particle_filter/number_of_particles', number_of_particles)

    filtered_pose_topic = rospy.get_param('topics/filtered_pose', 'filtered_pose')
    kinect_beacon_topic = rospy.get_param('topics/kinect_beacon_points', 'kinect_beacon_points')
    lidar_beacon_topic = rospy.get_param('topics/lidar_beacon_points', 'lidar_beacon_points')
    aruco_pose_topic = rospy.get_param('topics/aruco_pose', 'aruco_pose')
    command_topic = rospy.get_param('topics/command', 'cmd_vel')
    
    pub = rospy.Publisher(filtered_pose_topic, Pose) #Create our publisher to publish the pose we determine
    #Subscribe to the topics we need
    rospy.Subscriber(kinect_beacon_topic, localizationPoints, kinectBeaconCallback) #Subscribe to kinect beacon location
    rospy.Subscriber(lidar_beacon_topic, localizationPoints, lidarBeaconCallback) #Subscribe to lidar beacon location
    rospy.Subscriber(aruco_pose_topic, TransformStamped, arucoPoseCallback) #Subscribe to aruco pose
    rospy.Subscriber(command_topic, Twist, commandCallback) #Subscribe to the command issued
    
    #Specify beacon locations
    left_beacon = ( - beacon_width / 2.0, 0)
    right_beacon = ( beacon_width / 2.0, 0)
    
    #Initialize timing sum
    delta_t_sum = 0.0

    #Initialize particles 
    initialize_particles(number_of_particles)

    #Set iteration counter
    iteration = 0

    #Main filtering loop
    while not rospy.is_shutdown():

        #Keep track of time this iteration takes
        before = rospy.get_rostime()
        
        #Update the particles
        update_particles(iteration)

        #Get the filtered pose
        filtered_pose = compute_filtered_pose()
        #Publish it
        pub.publish(filtered_pose)
        
        #Increment our iteration counter
        iteration = iteration + 1
        
        #Figure our how long iteration took, keep track of stats
        #This gives us our delta_t for prediction step
        after = rospy.get_rostime()
        duration = after - before
        cur_delta_t = duration.to_sec()
        delta_t_sum = delta_t_sum + cur_delta_t
        #print 'Filter iteration took: ', delta_t_sum / float(iteration), 'seconds'

