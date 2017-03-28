#!/usr/bin/env python2

import rospy, signal, atexit, math, time, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool

'''
Proof of concept/sandbox module for trying out different ways to process lidar data.
'''

####### Default Global Values (stars lab motor tube testing) #######
#SCAN_TOPIC = "scan"
#POST_DIST = 1.2     # Distance posts are apart from one another on beacon
#POST_DIST_ERR = 0.2    #0.025  # Error allowed in post distance
MAX_RANGE = 15#1.25     # Max Scan range to consider
LARGE_NUMBER = 9999999  # Arbitrarily large number

#POST_WIDTH = 0.1          # Expected width of post
#POST_WIDTH_ERR = 0.1  # Error allowed in post width

#LEFT_POST_LOC = (0.65, 0)     # Global coordinate of left post
#RIGHT_POST_LOC = (-0.65, 0)    # Global coordinate of right post
#####################################

class LaserObject(object):
    '''
    Data structure used to hold objects detected in laser scans 
    '''
    def __init__(self):
        self.left_edge  = None  # index of left edge
        self.right_edge = None  # index of right edge
        self.length     = None  # obj length
        self.centroid   = None  # index of object centroid
        self.distance   = None  # centroid range in scan
        self.angle      = None  # centroid angle in scan

    def process(self, scan_msg):
        '''
        Given scan_msg and left and right edges, calculate:
         - length
         - centroid 
         - distance 
         - angle 
        '''
        assert self.left_edge != None and self.right_edge != None, "Object edges not set.  Cannot process."
        arc_ang = (self.left_edge - self.right_edge) * scan_msg.angle_increment
        right_dist = scan_msg.ranges[self.right_edge]
        left_dist = scan_msg.ranges[self.left_edge]
        try:
            self.length = math.sqrt((right_dist**2 + left_dist**2) - (2 * right_dist * left_dist * math.cos(arc_ang)))
        except:
            self.length = 0
        self.centroid = int(abs(self.right_edge - self.left_edge) / 2) + self.right_edge
        try:
            self.distance = scan_msg.ranges[self.centroid]
        except:
            self.distance = 0
        self.angle = self.centroid * scan_msg.angle_increment

class Beacon(object):
    '''
    Data structure used to store information on beacon found in laser scan 
    '''
    def __init__(self, right_post, left_post, actual_dist, err):
        self.right_post = right_post    # LaserObject that stores right post
        self.left_post  = left_post     # LaserObject that stores left post
        self.actual_dist = actual_dist  # Actual distance between right and left post
        self.err = err                  # (POST_DIST) - |DIST(right_post, left_post)|


class BeaconLocalizer(object):

    def __init__(self):
        #global SCAN_TOPIC, POST_DIST, LEFT_POST_LOC, RIGHT_POST_LOC
        #global POST_WIDTH, POST_WIDTH_ERR, POST_DIST_ERR
        '''
        Lidar Processor constructor
        '''
        rospy.init_node("beacon_localizer")

        self.listener = tf.TransformListener()

        ###################################
        # Load beacon localization params
        ###################################
        self.SCAN_TOPIC = rospy.get_param("beacon_localization/scan_topic")
        self.POST_DIST = rospy.get_param("beacon_localization/post_distance")
        self.POST_DIST_ERR = rospy.get_param("beacon_localization/post_distance_err")
        self.POST_WIDTH = rospy.get_param("beacon_localization/post_width")
        self.POST_WIDTH_ERR = rospy.get_param("beacon_localization/post_width_err")
        loc = rospy.get_param("beacon_localization/left_post_loc")
        self.LEFT_POST_LOC = (float(loc[0]), float(loc[1]))
        loc = rospy.get_param("beacon_localization/right_post_loc")
        self.RIGHT_POST_LOC = (float(loc[0]), float(loc[1]))
        self.BEACON_LOST_TOPIC = rospy.get_param("topics/beacon_lost", "beacon_lost")
        print("POST DIST: " + str(self.POST_DIST))
        ###################################

        ###################################
        # Load topic names
        ###################################
        ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose")

        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)

        #self.vis_scan_pub = rospy.Publisher("vis_scan", LaserScan, queue_size = 10)
        self.pose_pub = rospy.Publisher(ROBOPOSE_TOPIC, PoseStamped, queue_size = 10)
        self.beacon_lost_pub = rospy.Publisher(self.BEACON_LOST_TOPIC, Bool, queue_size = 10)

        self.current_pose = PoseStamped()
        self.current_scan = LaserScan() # current scan message
        self.received_scan = False      # True if we've received a new scan, false if not

        atexit.register(self._exit_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

        self.robot_location = (0, 0)    # Stores current robot location

    def scan_callback(self, data):
        '''
        This function is called everytime a message is transmitted over /scan topic
        '''
        # Update current scan
        self.current_scan = data
        # Set received scan flag to True
        self.received_scan = True

    def run(self):
        '''
        Main work loop.
        '''
        rate = rospy.Rate(10)
        this_scan = []
        while not rospy.is_shutdown():
            if self.received_scan:
                self.received_scan = False 
                this_scan = self.current_scan
                stime = time.time()
                self.process_scan(this_scan)
                etime = time.time()
                # print("Calc time: " + str(etime - stime))

            rate.sleep()

    def correct_dist(self, dist):
        '''
        Given a distance, clip distance to MAX_RANGE if longer than MAX_RANGE
        '''
        return dist if dist <= MAX_RANGE else MAX_RANGE

    def obj_dist(self, r_obj, l_obj):
        '''
        Given two objects (right object and left object), use law of cosines to calc 
         distance between the two.
        '''
        r = r_obj.distance
        l = l_obj.distance
        theta = l_obj.angle - r_obj.angle
        dist = math.sqrt(math.pow(r, 2) + math.pow(l, 2) - (2 * r * l * math.cos(theta)))
        return dist

    def process_scan(self, scan_msg):
        '''
        given a scan msg, process 
        '''
        current_angle = scan_msg.angle_min   # current angle will always have current angle (in lidar space)
        good_orientation = False 
        good_position = False

        ##################################
        # Visualization message (used to visualize software imposed laser range limit)
        # vis_scan = LaserScan()
        # vis_scan = scan_msg
        # vis_scan.range_max = MAX_RANGE
        # self.vis_scan_pub.publish(vis_scan)
        ##################################


        ####################### 
        # Pick out objects from scan
        #######################
        last_point = 0          # Previous point
        edge_thresh = 0.2       # thresh for edge detection
        scan_obj = LaserObject()
        scan_objs = []
        # print("======================")
        # loop through each laser scan
        for i in xrange(0, len(scan_msg.ranges)):
            # get corrected previous distance
            last_dist = self.correct_dist(scan_msg.ranges[last_point])
            # get corrected current distance
            cur_dist = self.correct_dist(scan_msg.ranges[i])
            # calculate change
            change = cur_dist - last_dist
            if abs(change) > edge_thresh and change < 0:
                # found a right edge
                scan_obj = LaserObject()
                scan_obj.right_edge = i
            elif abs(change) > edge_thresh and change > 0:
                # found a left edge
                if scan_obj.right_edge != None:
                    # make sure we've found a right edge already before this left edge
                    scan_obj.left_edge = last_point
                    scan_obj.process(scan_msg)
                    # Make sure object is of expected length
                    # print("Potential obj Length: " + str(scan_obj.length))
                    if (scan_obj.length >= self.POST_WIDTH - self.POST_WIDTH_ERR) and (scan_obj.length <= self.POST_WIDTH + self.POST_WIDTH_ERR):
                        scan_objs.append(scan_obj)
                    scan_obj = LaserObject()
                else:
                    scan_obj = LaserObject()

            # update current angle
            current_angle += scan_msg.angle_increment
            # update last point
            last_point = i
        # print("num objects: " + str(len(scan_objs)))
        # for obj in scan_objs:
            # print("~ OBJ")
            # print("    ~ Angle: " + str(math.degrees(obj.angle)))
            # print("    ~ Length: " + str(obj.length))
        ######################
        # Find the beacon (two posts distanced a known distance apart)
        ######################
        beacon = None
        min_beacon_err = LARGE_NUMBER
        for ri in xrange(0, len(scan_objs)):
            for li in xrange(ri  + 1, len(scan_objs)):
                r_obj = scan_objs[ri] # Grab right object for easy use
                l_obj = scan_objs[li] # Grab left object for easy use
                dist = self.obj_dist(r_obj, l_obj)  # calculate distance between right and left objects
                # check if dist indicates these two objects are a potential beacon
                # print("==== OBJ DIST ====")
                if (dist >= (self.POST_DIST - self.POST_DIST_ERR)) and (dist <= (self.POST_DIST + self.POST_DIST_ERR)):
                    beacon_err = abs(dist - self.POST_DIST)
                    if beacon_err < min_beacon_err:
                        beacon = Beacon(right_post = r_obj, left_post = l_obj, actual_dist = dist, err = beacon_err)
                        min_beacon_err = beacon_err
                        # print("Potential Beacon (err: " + str(beacon_err))
                ## Debugging/verbose information
                # print("Right Obj: (Centroid: %d, Angle: %f)" % (r_obj.centroid, math.degrees(r_obj.angle)))
                # print("Left Obj: (Centroid: %d, Angle: %f)" % (l_obj.centroid, math.degrees(l_obj.angle)))
                # print("Distance: " + str(self.obj_dist(r_obj, l_obj)))
    
        ## More debugging/verbose information
        if beacon != None:
            pass
            # print("~~~ BEACON ~~~")
            # print("(Centroid: %d, Angle: %f) ------ (Centroid %d, Angle %f)" % (beacon.left_post.centroid, math.degrees(beacon.left_post.angle), beacon.right_post.centroid, math.degrees(beacon.right_post.angle)))
            # print("Distance: " + str(beacon.actual_dist) + " (err: " + str(beacon.err) + ")")
        else:
            # print("~~~ BEACON ~~~")
            # print("Failed to find.")
            beacon_lost = Bool(True)
            self.beacon_lost_pub.publish(beacon_lost)
            return
        
        ###########################
        # Localize!
        ###########################
        # calculate global position
        try:
            xloc = (beacon.left_post.distance**2 - beacon.right_post.distance**2 - self.LEFT_POST_LOC[0]**2 + self.RIGHT_POST_LOC[0]**2) / (2*(self.RIGHT_POST_LOC[0] - self.LEFT_POST_LOC[0]))
            yloc = math.sqrt(beacon.right_post.distance**2 - (xloc - self.RIGHT_POST_LOC[0])**2)
        except:
            pass# print("Failed to calculate global position.")
        else:
            self.robot_location = (xloc, yloc)
            # print("ROBOT LOCATION: (%f, %f)" % (self.robot_location[0], self.robot_location[1]))
            good_position = True
        # calculate orientation
        try:
            #cosine rule, cos A = (a**2+b**2+c**2)/(2bc)
            alpha = math.acos((beacon.actual_dist**2 + beacon.left_post.distance**2 - beacon.right_post.distance**2) / (2 * beacon.actual_dist * beacon.left_post.distance))
        except:
            pass# print("Failed to calculate orientation")
        else:
            alphaOpp = math.pi - alpha
            theta = beacon.left_post.angle - math.pi / 2
            globOrient = alphaOpp - theta
            robOrient = globOrient - math.pi / 2
            # print("Global Orientation: %f deg" % (math.degrees(globOrient)))
            # print("Robot Orientation: %f deg" % (math.degrees(robOrient)))
            good_orientation = True
        #################################
        # Build Pose message and publish
        #################################
        if good_orientation and good_position:
            pose = PoseStamped() # GLOBAL
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "rear_lidar"
            # set position
            pose.pose.position.x = self.robot_location[0]
            pose.pose.position.y = self.robot_location[1]
            pose.pose.position.z = 0
            # set orientation
            roll = 0
            pitch = 0
            #yaw = robOrient
            yaw = globOrient
            quat = quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            # update current pose
            self.current_pose = pose 


#             pose_mag = beacon.left_post.distance #math.sqrt(pose.pose.position.y**2 + pose.pose.position.x**2)
#             pose_ang = beacon.left_post.angle#angle
#             x1 = beacon.left_post.distance * math.sin(beacon.left_post.angle)
#             y1 = beacon.left_post.distance * math.cos(beacon.left_post.angle)
#             x2 = beacon.right_post.distance * math.sin(beacon.right_post.angle)
#             y2 = beacon.right_post.distance * math.cos(beacon.right_post.angle)

#             transformX = (x1+x2)/2
#             transformY = -(y1+y2)/2

#             br = tf.TransformBroadcaster()
#             br.sendTransform((transformX, transformY, 0), quaternion_from_euler(0, 0, globOrient), rospy.Time.now(), "origin", "base_laser_link")#"rear_lidar_pos_global" )
            #(trans,rot) = self.listener.lookupTransform('base_laser_link', 'odom', rospy.Time(0))
            pose_mag = math.sqrt(pose.pose.position.y**2 + pose.pose.position.x**2)
            pose_ang = globOrient
            transformX = pose.pose.position.x #+ trans[0] #pose_mag * math.sin(pose_ang)
            transformY = pose.pose.position.y #+ trans[1] #pose_mag * math.cos(pose_ang)
            br = tf.TransformBroadcaster()
            br.sendTransform((transformX, transformY, 0), quaternion_from_euler(0, 0, globOrient), rospy.Time.now(), "base_link", "origin")#"rear_lidar_pos_global" )
            # br = tf.TransformBroadcaster()
            # br.sendTransform((-3, -3, -0.5), quaternion_from_euler(0, 0, 0), rospy.Time.now(), "map_origin", "origin" )

            # publish pose
            # print("Publishing pose.")
            beacon_lost = Bool(False)
            self.beacon_lost_pub.publish(beacon_lost)
            self.pose_pub.publish(pose)

        

    def _signal_handler(self, signal, frame):
        '''
        Called when ctr-c signal is received
        '''
        exit()

    def _exit_handler(self):
        '''
        Called on script exit 
        '''
        pass

def meters_to_inches(val):
    '''
    THIS IS FOR VISUALIZATION PURPOSES ONLY 
    DO NOT USE INCHES FOR ANYTHING
    '''
    return val * 39.370


if __name__ == "__main__":
    processor = BeaconLocalizer()
    processor.run()

