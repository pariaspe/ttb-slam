#!/usr/bin/env python

import rospy
import tf

from math import pi

rospy.init_node('add_world')
rate = rospy.Rate(10.0)

quat_world = tf.transformations.quaternion_from_euler(0, 0, pi)
quat_map = tf.transformations.quaternion_from_euler(0, 0, pi)
br = tf.TransformBroadcaster()

transform = tf.Transformer
while not rospy.is_shutdown():
    br.sendTransform((6.0, 6.0, 0.0),
                 quat_world,
                 rospy.Time.now(),
                 'odom',
                 'world')
    br.sendTransform((6.0, 6.0, 0.0),
                 quat_map,
                 rospy.Time.now(),
                 'map',
                 'world')
    
    rate.sleep()

