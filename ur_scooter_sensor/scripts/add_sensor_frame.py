#!/usr/bin/env python
'''Broadcasts the transform for the sensor.

Credits:
- http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29

Assembled: Northeastern University, 2015
'''

import rospy
import tf
from visualization_msgs.msg import Marker

if __name__ == '__main__':
  
  rospy.init_node('add_sensor_frame')
  br = tf.TransformBroadcaster()
  rate = rospy.Rate(30)
  pub = rospy.Publisher("/ground", Marker, queue_size=1)
  
  while not rospy.is_shutdown():
    # calibrated on 07/26/18
    br.sendTransform((0.104, 0.051, 0.05), (0, 0, 0, 1), rospy.Time.now(), "camera_link", "ee_link")

    #br.sendTransform((-0.357, 0.013, 0.421), (-0.1155, 0.1828, 0.5195, 0.8267), rospy.Time.now(),"camera_left_link", "base_link")
    br.sendTransform((-0.283 ,-0.068 ,0.448), (-0.2422, 0.3978, 0.4705, 0.7495), rospy.Time.now(),"camera_left_link", "base_link" )
    #br.sendTransform((-0.255 ,-0.054 ,0.456), (-0.2422, 0.3978, 0.4705, 0.7495), rospy.Time.now(),"camera_left_link", "base_link" )

    br.sendTransform((-0.353, -0.008, 0.398), (-0.2422, 0.3978, 0.4705, 0.7495), rospy.Time.now(), "camera_left_down_link", "base_link")

    # Structure up on the right bar
    br.sendTransform((0.253 ,-0.078 ,0.428), (0.4821 ,0.743 ,-0.2524 ,0.3897), rospy.Time.now(), "camera_right_link", "base_link")

    # Structure down on the right bar
    br.sendTransform((0.351, -0.032, 0.37), (0.4821, 0.743, -0.2524, 0.3897), rospy.Time.now(), "camera_right_down_link", "base_link")


    # Structure on the left up bar
    # calibrated on 08/10/18
    br.sendTransform((-0.277, -0.148, 0.839), (0.7515, 0.5325, -0.3108, 0.2348), rospy.Time.now(), "camera_up_link", "base_link")


    rate.sleep()
