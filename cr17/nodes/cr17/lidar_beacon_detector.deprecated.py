#!/usr/bin/env python2

import rospy, signal, atexit, math, time, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from classic_robot.msg import localizationPoint, localizationPoints

'''
Proof of concept/sandbox module for trying out different ways to process lidar data.
'''

####### Default Global Values (stars lab motor tube testing) #######
SCAN_TOPIC = "scan"
POST_DIST = 1.2     # Distance posts are apart from one another on beacon
POST_DIST_ERR = 0.2    #0.025  # Error allowed in post distance
MAX_RANGE = 15#1.25     # Max Scan range to consider
LARGE_NUMBER = 9999999  # Arbitrarily large number

POST_WIDTH = 0.1          # Expected width of post
POST_WIDTH_ERR = 0.1  # Error allowed in post width

LEFT_POST_LOC = (0.65, 0)     # Global coordinate of left post
RIGHT_POST_LOC = (-0.65, 0)    # Global coordinate of right post
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
        global SCAN_TOPIC, POST_DIST, LEFT_POST_LOC, RIGHT_POST_LOC
        global POST_WIDTH, POST_WIDTH_ERR, POST_DIST_ERR
        '''
        Lidar Processor constructor
        '''
        rospy.init_node("lidar_beacon_detector")

        self.listener = tf.TransformListener()

        ###################################
        # Load beacon localization params
        ###################################
        SCAN_TOPIC = "base_scan"#rospy.get_param("beacon_localization/scan_topic", SCAN_TOPIC)
        POST_DIST = rospy.get_param("beacon_localization/post_distance", POST_DIST)
        PORT_DIST_ERR = rospy.get_param("beacon_localization/post_distance_err", POST_DIST_ERR)
        POST_WIDTH = rospy.get_param("beacon_localization/post_width", POST_WIDTH)
        POST_WIDTH_ERR = rospy.get_param("beacon_localization/post_width_err", POST_WIDTH_ERR)
        loc = rospy.get_param("beacon_localization/left_post_loc", LEFT_POST_LOC)
        LEFT_POST_LOC = (float(loc[0]), float(loc[1]))
        loc = rospy.get_param("beacon_localization/right_post_loc", RIGHT_POST_LOC)
        RIGHT_POST_LOC = (float(loc[0]), float(loc[1]))
        BEACON_LOST_TOPIC = rospy.get_param("topics/beacon_lost", "beacon_lost")
        BEACON_POINT_TOPIC = rospy.get_param("topics/lidar_beacon_points", "lidar_beacon_points")
        # print("POST DIST: " + str(POST_DIST))
        ###################################

        ###################################
        # Load topic names
        ###################################
        ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose", "beacon_localization_pose")

        rospy.Subscriber(SCAN_TOPIC, LaserScan, self.scan_callback)

        self.vis_scan_pub = rospy.Publisher("vis_scan", LaserScan, queue_size = 10)
        #self.pose_pub = rospy.Publisher(ROBOPOSE_TOPIC, PoseStamped, queue_size = 10)
        #self.beacon_lost_pub = rospy.Publisher(BEACON_LOST_TOPIC, Bool, queue_size = 10)
        self.beacon_point_pub = rospy.Publisher(BEACON_POINT_TOPIC, localizationPoints, queue_size = 10)

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
                    if (scan_obj.length >= POST_WIDTH - POST_WIDTH_ERR) and (scan_obj.length <= POST_WIDTH + POST_WIDTH_ERR):
                        scan_objs.append(scan_obj)
                    scan_obj = LaserObject()
                else:
                    scan_obj = LaserObject()

            # update current angle
            current_angle += scan_msg.angle_increment
            # update last point
            last_point = i
        # print("num objects: " + str(len(scan_objs)))
        
        pos_beacons = localizationPoints()
        for obj in scan_objs:
            pos_beacon = localizationPoint(distance = obj.distance,angle = obj.angle, confidence = 1) #TODO: this should probably be changed

            pos_beacons.points.append(pos_beacon)

        self.beacon_point_pub.publish(pos_beacons)

        

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

