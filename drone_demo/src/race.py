#! /usr/bin/env python3

import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class RunRace(object):

  def __init__(self):
    self.state = "TAKEOFF"
    self.ctrl_c = False
    self.rate = rospy.Rate(10)

    # define the different publishers, subscribers, and messages that will be used
    rospy.Subscriber("/drone/down_camera/image_raw", Image, self.down_camera_cb)
    self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self._move_msg = Twist()
    self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    self._takeoff_msg = Empty()
    self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
    self._land_msg = Empty()
    self.bridge = CvBridge()
    self.rate = rospy.Rate(10)
    self.lower_bound = np.array([72, 72, 72])
    self.upper_bound = np.array([90,90,90])
    
  
  def publish_once_in_cmd_vel(self, cmd):
    """
    This is because publishing in topics sometimes fails teh first time you publish.
    In continuos publishing systems there is no big deal but in systems that publish only
    once it IS very important.
    """
    while not self.ctrl_c:
        connections = self._pub_cmd_vel.get_num_connections()
        if connections > 0:
            self._pub_cmd_vel.publish(cmd)
            rospy.loginfo("Publish in cmd_vel...")
            break
        else:
            self.rate.sleep()
            
  # function that stops the drone from any movement
  def stop_drone(self):
    rospy.loginfo("Stopping...")
    self._move_msg.linear.x = 0.0
    self._move_msg.angular.z = 0.0
    self.publish_once_in_cmd_vel(self._move_msg)
        
  # function that makes the drone turn 90 degrees
  def turn_drone(self, speed):
    rospy.loginfo("Turning...")
    self._move_msg.linear.x = 0.0
    self._move_msg.angular.z = speed
    self.publish_once_in_cmd_vel(self._move_msg)
    
  # function that makes the drone move forward
  def move_forward_drone(self, speed):
    rospy.loginfo("Moving forward...")
    self._move_msg.linear.x = speed
    self._move_msg.angular.z = 0.0
    self.publish_once_in_cmd_vel(self._move_msg)
  
  def move_drone(self, speeds):
    rospy.loginfo("Moving...")
    self._move_msg.linear.x = speeds[0]
    self._move_msg.linear.y = speeds[1]
    self._move_msg.linear.z = speeds[2]
    self.publish_once_in_cmd_vel(self._move_msg)

  def takeoff(self):
    # make the drone takeoff
    i=0
    while not i == 3:
        self._pub_takeoff.publish(self._takeoff_msg)
        rospy.loginfo('Taking off...')
        time.sleep(1)
        i += 1
    self.move_drone((0,0,1.2))
    time.sleep(0.6)
    self.move_drone((0,0,-1))
    time.sleep(0.1)
    self.move_drone((0,0,0))
    time.sleep(0.2)
    self.turn_drone(-0.1)
    time.sleep(2.8)
    self.turn_drone(0)
    time.sleep(0.5)
    self.move_drone((1.0,0,0))
    time.sleep(2.0)
    self.move_drone((0,0,0))
    self.state = "HOVER"

  def land(self):
    i=0
    while not i == 3:
        self._pub_land.publish(self._land_msg)
        rospy.loginfo('Landing...')
        time.sleep(1)
        i += 1
    self.state = "HOVER"

  def down_camera_cb(self, msg: Image) -> None:
      img = self.bridge.imgmsg_to_cv2(msg)
      mask = cv2.inRange(img, self.lower_bound, self.upper_bound)
      cv2.imshow("stream", mask)
      cv2.waitKey(1)
    
  def run_race(self):
    # this callback is called when the action server is called.
    # this is the function that computes the Fibonacci sequence
    # and returns the sequence to the node that called the action server
    
    # helper variables
    r = rospy.Rate(1)

    while(True):
      if self.state == "TAKEOFF":
        self.takeoff()
      elif self.state == "HOVER":
        self.move_drone((0,0,0))
      elif self.state == "LAND":
        self.land()
    
      
if __name__ == '__main__':
  rospy.init_node('race')
  run_race = RunRace()
  try:
      run_race.run_race()
  except rospy.ROSInterruptException:
      pass