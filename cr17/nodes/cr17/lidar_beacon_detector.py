#!/usr/bin/env python2

import rospy, signal, atexit, math, time, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from cr17.msg import localizationPoint, localizationPoints
from laser_object import LaserObject


'''
Proof of concept/sandbox module for trying out different ways to process lidar data.
'''


class LidarBeaconDetector(object):

    def __init__(self):
        '''
        Lidar Processor constructor
        '''
        rospy.init_node("lidar_beacon_detector")

        self.listener = tf.TransformListener()

        ###################################
        # Load beacon localization params
        ###################################
        self.POST_WIDTH = rospy.get_param("beacon_localization/post_width")
        self.POST_WIDTH_ERR = rospy.get_param("beacon_localization/post_width_err")
        self.MAX_RANGE = rospy.get_param("beacon_localization/max_range")
        self.BEACON_POINT_TOPIC = rospy.get_param("topics/lidar_beacon_points", "lidar_beacon_points")
        ###################################

        ###################################
        # Load topic names
        ###################################
        rospy.Subscriber("/scan_topic", LaserScan, self.scan_callback)

        self.vis_scan_pub = rospy.Publisher("vis_scan", LaserScan, queue_size = 10)
        self.beacon_point_pub = rospy.Publisher(self.BEACON_POINT_TOPIC, localizationPoints, queue_size = 10)

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
        return dist if dist <= self.MAX_RANGE else self.MAX_RANGE

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
    processor = LidarBeaconDetector()
    processor.run()

