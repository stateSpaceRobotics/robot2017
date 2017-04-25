#!/usr/bin/env python2

import rospy, signal, atexit, math, time, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from beacon import Beacon
from laser_object import LaserObject
from cr17.msg import localizationPoint, localizationPoints


'''
Proof of concept/sandbox module for trying out different ways to process lidar data.
'''

####### Default Global Values (stars lab motor tube testing) #######
MAX_RANGE = 15#1.25     # Max Scan range to consider
LARGE_NUMBER = 9999999  # Arbitrarily large number
#####################################


class BeaconLocalizer(object):
    def __init__(self):
        '''
        Lidar Processor constructor
        '''
        rospy.init_node("beacon_localizer")

        self.listener = tf.TransformListener()

        ###################################
        # Load beacon localization params
        ###################################
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

        self.latest_hokuyo_scan = []
        self.latest_sick_scan = []

        rospy.Subscriber("/hokuyo_lidar_beacon_points", localizationPoints, self.hokuyo_callback)
        rospy.Subscriber("/sick_lidar_beacon_points", localizationPoints, self.sick_callback)

        self.pose_pub = rospy.Publisher(ROBOPOSE_TOPIC, PoseStamped, queue_size = 10)
        self.beacon_lost_pub = rospy.Publisher(self.BEACON_LOST_TOPIC, Bool, queue_size = 10)

        atexit.register(self._exit_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

        self.robot_location = (0, 0)    # Stores current robot location

        self.hokuyo_transform = None
        self.sick_transform = None

    def hokuyo_callback(self, data):
        for i, _ in enumerate(data.points):
            if data.points[i].angle > math.pi:
                data.points[i].angle -= math.pi
            else:
                data.points[i].angle += math.pi
        self.latest_hokuyo_scan = data.points

    def sick_callback(self, data):
        self.latest_sick_scan = data.points

    def run(self):
        '''
        Main work loop.
        '''
        while(self.hokuyo_transform is None or self.sick_transform is None):
            try:
                self.hokuyo_transform = self.listener.lookupTransform("/hokuyo_lidar", "/lidar_mount", rospy.Time(0))
                self.sick_transform = self.listener.lookupTransform("/sick_lidar", "/lidar_mount", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                continue

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.process_beacon_points()
            rate.sleep()

    def obj_dist(self, object1, object2):
        '''
        Given two objects (right object and left object), use law of cosines to calc 
         distance between the two.
        '''
        object1Distance = object1.distance
        object2Distance = object2.distance
        theta = object2.angle - object1.angle
        dist = math.sqrt(object1Distance**2 + object2Distance**2 - (2 * object1Distance * object2Distance * math.cos(theta)))
        return dist

    def wrap_angle(self, angle):
        r_angle = angle
        while r_angle <= -math.pi:
            r_angle += 2 * math.pi
        while r_angle >= math.pi:
            r_angle -= 2 * math.pi
        return r_angle

    def process_beacon_points(self):
        # rospy.logerr("SICK objects: " + str(len(self.latest_sick_scan)))
        # for obj in self.latest_sick_scan:
        #     rospy.logerr("~ OBJ")
        #     rospy.logerr("    ~ Angle: " + str(math.degrees(obj.angle)))
        #     rospy.logerr("    ~ Distance: " + str(obj.distance))
        #
        # rospy.logerr("Hokuyo objects: " + str(len(self.latest_hokuyo_scan)))
        # for obj in self.latest_hokuyo_scan:
        #     rospy.logerr("~ OBJ")
        #     rospy.logerr("    ~ Angle: " + str(math.degrees(obj.angle)))
        #     rospy.logerr("    ~ Distance: " + str(obj.distance))

        scan_objs = self.latest_sick_scan + self.latest_hokuyo_scan

        ######################
        # Find the beacon (two posts distanced a known distance apart)
        ######################
        good_orientation = False
        good_position = False
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
    
        ## More debugging/verbose information
        if beacon != None:
            if not 0 < (beacon.left_post.angle - beacon.right_post.angle) < math.pi and not (beacon.left_post.angle - beacon.right_post.angle) < -math.pi:
                # rospy.logerr("swapping left and right beacons: %f" % math.degrees(beacon.left_post.angle - beacon.right_post.angle))
                beacon.left_post, beacon.right_post = beacon.right_post, beacon.left_post

            # rospy.logerr("~~~ BEACON ~~~")
            # rospy.logerr("Left Distance: %f, Angle: %f" % (beacon.left_post.distance, math.degrees(beacon.left_post.angle)))
            # rospy.logerr("Right Distance %f, Angle %f" % (beacon.right_post.distance, math.degrees(beacon.right_post.angle)))
            # rospy.logerr("Distance between posts: " + str(beacon.actual_dist) + " (err: " + str(beacon.err) + ")")
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
            xloc = (-beacon.left_post.distance**2 + beacon.right_post.distance**2 - self.LEFT_POST_LOC[0]**2 + self.RIGHT_POST_LOC[0]**2) / (2*(self.RIGHT_POST_LOC[0] - self.LEFT_POST_LOC[0]))
            yloc = math.sqrt(beacon.right_post.distance**2 - (xloc - self.RIGHT_POST_LOC[0])**2)
        except:
            pass
        else:
            self.robot_location = (xloc, yloc)
            # rospy.logerr("ROBOT LOCATION: (%f, %f)" % (self.robot_location[0], self.robot_location[1]))
            good_position = True

        # calculate orientation
        try:
            #cosine rule, cos A = (a**2+b**2-c**2)/(2ab)
            alpha = math.acos((beacon.actual_dist**2 + beacon.left_post.distance**2 - beacon.right_post.distance**2) / (2 * beacon.actual_dist * beacon.left_post.distance))
        except:
            pass
        else:
            alphaOpp = math.pi - alpha
            theta = beacon.left_post.angle
            globOrient = alphaOpp - theta
            globOrient = self.wrap_angle(globOrient)
            # rospy.logerr("Global Orientation: %f deg" % (math.degrees(globOrient)))
            good_orientation = True

        # rospy.logerr("beaconDist: " + str(math.degrees(beacon.actual_dist)))
        # rospy.logerr("alpha: " + str(math.degrees(alpha)))
        # rospy.logerr("theta: " + str(math.degrees(theta)))
        # rospy.logerr("globOrient: " + str(math.degrees(globOrient)))
        robOrient = globOrient - math.pi / 2
        # print("Global Orientation: %f deg" % (math.degrees(globOrient)))
        # print("Robot Orientation: %f deg" % (math.degrees(robOrient)))


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
            yaw = globOrient
            quat = quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]


            transformX = pose.pose.position.x
            transformY = pose.pose.position.y
            br = tf.TransformBroadcaster()
            br.sendTransform((transformX, transformY, 0), quaternion_from_euler(0, 0, globOrient), rospy.Time.now(), "base_link", "odom")

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

