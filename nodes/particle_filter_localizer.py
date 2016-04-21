#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math
import random
from labutils import *
from sensormodel import *
from motionmodel import *
from discretemap import *
from distancelut import *

#####################
# BEGIN Global Variable Definitions
particles = []        #Our list of particles
delta_t = 0.5         #Keep track of how long between predictions
delta_t_sum = 0.0     #How long between predictions as an average of previous iteration lengths
delta_t_num = 0       #How many previous iterations went into the average
t_weight =0
kinect_beacon = None  # The relative location of the beacons (from kinect)
lidar_beacon = None   # The relative location of the beacons (from lidar)
aruco_pose = None # Our pose estimate (from aruco)
imu_pose = None # Our pose estimate (from imu)
command = None # The most recent drive command

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
    aruco_pose = data
    
def imuPoseCallback(data):
    global imu_pose
    imu_pose = data
    
#Command callback, store as global variable
def commandCallback(data):
    global command
    command = data

#Robot position callback, extract pose and save as global
def robotCallback(data):
    #This function updates the robots position and yaw, based on the ground truth (this is simply to display the true robot location on the images)
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

############################
##### END CALLBACK FUNCTIONS  
############################

##########################
# BEGIN PARTICLE FILTER CODE
##########################

#Initialize the global list of particles with n particles
#Randomly distributed around our initial pose
def initialize_particles(n):
    global particles

    x=0
    while x !=n:
	i=get_random_particle()
	particles.append(i)
	x=x+1
    #print particles
   
#Advance particles forward in delta_t in time, using motion model
def advance_particles():
    global particles
    global delta_t
    global command

    #Create Control Object for motion model to use
    u = Control(command.linear.x, command.angular.z)

    #Create motionmodel params objec
    vp = VelocityParams(0.1,0.1,0.1,0.1,0.1,0.1,delta_t)

    #Advance each particle
    for i in range(len(particles)):
        old_pose = Pose(particles[i].x, particles[i].y, particles[i].theta)
        
        #Use motion model to get new pose
        new_pose = sample_motion_model_velocity(u, old_pose, vp)
        
        #Assign this new pose to the particle
        particles[i].x = new_pose.x
        particles[i].y = new_pose.y
        particles[i].theta = new_pose.theta
                       
#Check to see if a particle is on the map
def particle_on_map(p):
    global discrete_map
    epsilon = 0.05

    if p.x < -discrete_map.map_width/2.0 + epsilon or p.x > discrete_map.map_width/2.0 - epsilon:
        return False
    if p.y < -discrete_map.map_height/2.0 + epsilon or p.y > discrete_map.map_height/2.0 - epsilon:
        return False

    (gx,gy) = discrete_map.map_to_grid((p.x,p.y))
    if (gx,gy) in discrete_map.occupied:
        return False
    
    return True

#Assign each particle its weight
def get_particle_weights():
    global particles
    global t_weight
    t_weight = 0
    #Cycle through each particle and assign it a weight
    #The weight should be stored in particles[i].w
    for i in range(len(particles)):
        #See if particle is in an obstacle or off the map, if it is, it gets weight of 0.0
        if particle_on_map(particles[i]):
            #ASSIGN PARTICLE i its WEIGHT (save in particles[i].w)
            particles[i].w = get_scan_prob(particles[i])
	    t_weight = particles[i].w + t_weight
	    #print particles[i].w
	    #print j
	    #j = j+1
        else:
            particles[i].w = 0.0

#Get the next set of particles
#By resampling current ones according to weights, with replacement
def resample_particles():
    global particles
    total = 0
    global t_weight 
    new_particles=[]
    
    i = 0
    count =0
    while i!= 450: 
	#print 'i',i
	i=i+1
	j=0
	rand = random.uniform (0,t_weight)
	while j != 450 :
		#print j
		total=total + particles[j].w
		j=j+1
    		if  total >= rand :
			print 'j',j
			m= particles[j].x
			a=particles[j].y
			b=particles[j].theta
			c=particles[j].w
			#print 'total ',total
			#print 'rand ',rand
			#print 'c',c
			p = Particle(m,a,b,c)
			new_particles.append(p)
			#print 'particle',new_particles.append((m,a,b,c))
			print 'count',count
			count=count+1
			break
    x=0
    while x !=50:
	new_particles.append(get_random_particle())
	x=x+1
			

    #TODO
    #PART C: RESAMPLE PARTICLES
    #Using the weights of the particles, resample a new set of particles, 
    #(the same number as were in the previous set of particles)
    #Each time you choose one from the old set (particles), create a new one and add it to the 
    #new_particles list.  
    #Make sure that you also add in some number of random particles at each iteration (you can play with how many)
    #Look at the initialize particles code to see how to get a random particles easily
    #Set our new set of particles to be our set of particles
    

	
    #print t_weight
    particles = new_particles

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

    #Subscribe to the topics we need
    rospy.Subscriber("kinect_beacon_location", CustomTopic, kinectBeaconCallback) #Subscribe to kinect beacon location
    rospy.Subscriber("lidar_beacon_location", CustomTopic, lidarBeaconCallback) #Subscribe to lidar beacon location
    rospy.Subscriber("aruco_pose", Pose, arucoPoseCallback) #Subscribe to aruco pose
    rospy.Subscriber("imu_pose", Pose, imuPoseCallback) #Subscribe to imu pose
    rospy.Subscriber("cmd_vel",Twist,commandCallback) #Subscribe to the command issued

    #Declare needed global variables
    global delta_t
    global distance_LUT
    global discrete_map
    global robot
    global laser_data

    #Process arguments and world file name
    args = rospy.myargv(argv=sys.argv)
    discrete_map = DiscreteMap(args[1], 5)
    distance_LUT = DistanceLUT(discrete_map)

    #Initialize timing sum
    delta_t_sum = 0.0

    #Initialize particles 
    #We will start with 500 particles, but you can experiment with different numbers of particles
    initialize_particles(500)

    #Set iteration counter
    iteration = 0

    #Save an image of particles how often (in iterations)
    #Change this to influence how many images get saved out
    display_rate = 1
    

    #Main filtering loop
    while not rospy.is_shutdown():
        #Only proceed if we have received a laser scan
        if got_laser: 

            #Keep track of time this iteration takes
            before = rospy.get_rostime()
            
            #See if we should save out an image of the particles
            if ((iteration % display_rate) == 0 and iteration <50):
                sname = discrete_map.world_dir + '/' + 'pf_' + discrete_map.world_name + '_' + str(iteration).zfill(4) + '.png'
                discrete_map.display_particles(particles, robot, laser_data, sname)

	
            #Update the particles
            update_particles(iteration)

            #Increment our iteration counter
            iteration = iteration + 1
            
            #Figure our how long iteration took, keep track of stats
            #This gives us our delta_t for prediction step
            after = rospy.get_rostime()
            duration = after - before
            cur_delta_t = duration.to_sec()
            delta_t_sum = delta_t_sum + cur_delta_t
            delta_t = delta_t_sum / float(iteration)

