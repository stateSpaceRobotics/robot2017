#!/usr/bin/env python

import roslib
import rospy
import tf
from geometry_msgs.msg import TransformStamped
roslib.load_manifest('classic_robot')

br = tf.TransformBroadcaster()


def handle_transform(msg):
    global br
    pos = msg.transform.translation
    rot = msg.transform.rotation
    br.sendTransform((pos.x, pos.y, pos.z),
                     (rot.x, rot.y, rot.z, rot.w),
                     msg.header.stamp,
                     msg.child_frame_id,
                     "/%s" % msg.header.frame_id)

if __name__ == '__main__':
    rospy.init_node('aruco_tf_broadcaster')
    aruco_transform_topic = rospy.get_param(
        '~aruco_transform_topic',
        '/ar_multi_board/transform')
    rospy.Subscriber(aruco_transform_topic,
                     TransformStamped,
                     handle_transform)
    rospy.spin()
