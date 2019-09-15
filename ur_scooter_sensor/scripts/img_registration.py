#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
import tf2_ros
import tf

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import matplotlib.pyplot as plt
import sys

sys.path.append('/home/scooter/ur5_ws/src/ur_scooter_sensor/scripts')
from img_util import registerRGBToDepth


class img_registration():
  def __init__(self):
    rospy.init_node("img_registration")
    print('Initializing Node')
    # self.depth_img = '/camera_right_down/depth/image'
    # self.depth_camera_info = '/camera_right_down/depth/camera_info'
    # self.rgb_img ='/camera_right/rgb/image'
    # self.rgb_camera_info ='/camera_right/rgb/camera_info'

    # Set Subscribe Topic
    self.depth_img = '/' + rospy.get_param('~depth_name') + '/depth/image'
    self.depth_camera_info = '/' + rospy.get_param('~depth_name') + '/depth/camera_info'
    self.rgb_img = '/' + rospy.get_param('~rgb_name') + '/rgb/image'
    self.rgb_camera_info = '/' + rospy.get_param('~rgb_name') + '/rgb/camera_info'

    self.depth_msg = None
    self.rgb_msg = None
    self.depth_info = None
    self.rgb_info = None

    # Set Publish Rate
    self.r = rospy.Rate(30)

    # Set Publish Topic
    self.pub_img = rospy.Publisher('/' + rospy.get_param('~depth_name') + '/registered/rgb_image', Image, queue_size=1)
    self.pub_info = rospy.Publisher('/' + rospy.get_param('~depth_name') + '/registered/camera_info', CameraInfo,queue_size=1)
    self.pub_depth = rospy.Publisher('/' + rospy.get_param('~depth_name') + '/registered/depth_image', Image,queue_size=1)

    self.point_cloud = rospy.Publisher('/'+ rospy.get_param('~depth_name')+'/registered/points', PointCloud2, queue_size=1)

    self.bridge = CvBridge()
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    # define subscribers
    self.depth_sub = rospy.Subscriber(self.depth_img, Image, self.callbackDepth, queue_size=1)
    self.depth_info_sub = rospy.Subscriber(self.depth_camera_info, CameraInfo, self.callbackDepthInfo, queue_size=1)

    self.rgb_sub = rospy.Subscriber(self.rgb_img, Image, self.callbackRGB, queue_size=1)
    self.rgb_info_sub = rospy.Subscriber(self.rgb_camera_info, CameraInfo, self.callbackRGBInfo, queue_size=1)

    self.flag = 1

    # self.depth_sub.unregister()
    # self.depth_info_sub.unregister()
    # self.rgb_sub.unregister()
    # self.rgb_info_sub.unregister()

    print('Initializing Node Complete')

  def callbackDepth(self, msg):
    if not self.depth_msg:
      self.depth_msg = msg
      #print 'get depth'

  def callbackRGB(self, msg):
    if not self.rgb_msg:
      self.rgb_msg = msg
      #print 'get rgb'

  def callbackDepthInfo(self, msg):
    if not self.depth_info:
      self.depth_info = msg
      #print 'get depth info'

  def callbackRGBInfo(self, msg):
    if not self.rgb_info:
      self.rgb_info = msg
      #print 'get rgb info'

  def publish_img_info(self, reg_rgb_img):
    reg_rgb_img = reg_rgb_img.astype(np.uint8)
    img_msg = self.bridge.cv2_to_imgmsg(reg_rgb_img, "rgb8")
    img_msg.header = self.depth_info.header
    # publish registered rgb image
    self.pub_img.publish(img_msg)
    # publish depth camera infomation
    self.pub_info.publish(self.depth_info)
    # publish depth image
    self.pub_depth.publish(self.depth_msg)

    self.depth_msg = None
    self.rgb_msg = None
    self.depth_info = None
    self.rgb_info = None

    #print 'published image'

  def process(self):
    #print(self.point_cloud.get_num_connections())

    if self.point_cloud.get_num_connections() == 1 :
        if self.flag == 1:
            self.depth_sub = rospy.Subscriber(self.depth_img, Image, self.callbackDepth, queue_size=1)
            self.depth_info_sub = rospy.Subscriber(self.depth_camera_info, CameraInfo, self.callbackDepthInfo, queue_size=1)

            self.rgb_sub = rospy.Subscriber(self.rgb_img, Image, self.callbackRGB, queue_size=1)
            self.rgb_info_sub = rospy.Subscriber(self.rgb_camera_info, CameraInfo, self.callbackRGBInfo, queue_size=1)
            self.flag = 0

        #print('Get information from ' + rospy.get_param('~depth_name') + 'and' + rospy.get_param(('~rgb_name')))
        # get images, transform image messages to cv image (np array)
        while not self.depth_msg or not self.rgb_msg or not self.depth_info or not self.rgb_info:
          rospy.sleep(0.01)
        try:
          depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg)
          rgb_img = self.bridge.imgmsg_to_cv2(self.rgb_msg)

        except CvBridgeError as e:
          print e
        # get transformation between cameras
        while not rospy.is_shutdown():
          try:
            transformMsg = self.tfBuffer.lookup_transform(rospy.get_param('~rgb_name') + '_rgb_optical_frame',
                                                          rospy.get_param('~depth_name') + '_depth_optical_frame',
                                                          rospy.Time())
            break
          except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.01)
            continue
        translation = transformMsg.transform.translation
        pos = [translation.x, translation.y, translation.z]
        rotation = transformMsg.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        T = tf.transformations.quaternion_matrix(quat)
        T[0:3, 3] = pos
        depth_to_rgb = T
        # call my function to get registered rgb image
        reg_rgb_img = registerRGBToDepth(rgb_img, depth_img, self.rgb_info, self.depth_info, depth_to_rgb)
        # plot image
        # fig, axarr = plt.subplots(1, 3, figsize=(15, 5))
        # axarr[0].imshow(rgb_img)
        # axarr[1].imshow(depth_img)
        # axarr[2].imshow(reg_rgb_img)
        # fig.show()
        # plt.show()
        # publish
        self.publish_img_info(reg_rgb_img)
    else:
        self.depth_sub.unregister()
        self.depth_info_sub.unregister()
        self.rgb_sub.unregister()
        self.rgb_info_sub.unregister()

        self.flag = 1



def main():
  reg = img_registration()
  while not rospy.is_shutdown():
    # print(reg.i)

    reg.process()
    reg.r.sleep()
if __name__ == "__main__":
  main()


