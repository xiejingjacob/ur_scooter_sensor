#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import *
import roslaunch

# class SensorLaunch:
#   def __init__(self):
#     rospy.init_node('ur_scooter_sensor')
#     self.sub = rospy.Subscriber('sensor_launch', Bool, self.callback)
#     self.relaunchFlag = False
#     self.shutdownFlag = False
#     rospy.on_shutdown(self.shutdownCallback)
#
#   def callback(self, msg):
#     self.relaunchFlag = True
#
#   def shutdownCallback(self):
#     self.shutdownFlag = True
#
#   def run(self):
#     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#     roslaunch.configure_logging(uuid)
#     launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ur5/ur5_ws/src/ur_scooter/launch/sensor.launch"])
#     launch.start()
#     while True:
#       if self.shutdownFlag:
#         break
#
#       if not self.relaunchFlag:
#         rospy.sleep(0.1)
#       else:
#         launch.shutdown()
#         self.relaunchFlag = False
#
#         uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#         roslaunch.configure_logging(uuid)
#         launch = roslaunch.parent.ROSLaunchParent(uuid,
#                                                   ["/home/ur5/ur5_ws/src/ur_scooter/launch/sensor.launch"])
#         launch.start()
#
# sensorLaunch = SensorLaunch()
# sensorLaunch.run()

class SensorLaunch:
  def __init__(self, name):
    self.launchArgs = ['ur_scooter_2', name+'.launch']

    self.launch = self.generateLaunch()
    self.launch.start()

    self.rebootService = rospy.Service('/'+name+'/reboot', Trigger, self.rebootCallback)

  def generateLaunch(self):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    launchFile = roslaunch.rlutil.resolve_launch_arguments(self.launchArgs)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, launchFile)
    roslaunch.configure_logging(uuid)
    return launch

  def rebootCallback(self, req):
    self.launch.shutdown()
    self.launch = self.generateLaunch()
    self.launch.start()
    return TriggerResponse(success=True)

rospy.init_node('ur_scooter_sensor_left')

left = SensorLaunch('camera_left_down')

rospy.spin()

