#! /usr/bin/env python3

import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class RunRace(object):

  def __init__(self):
      
    self.ctrl_c = False
    self.rate = rospy.Rate(10)
    
  
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
    
  def run_race(self):
    # this callback is called when the action server is called.
    # this is the function that computes the Fibonacci sequence
    # and returns the sequence to the node that called the action server
    
    # helper variables
    r = rospy.Rate(1)
    
    # define the different publishers and messages that will be used
    self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self._move_msg = Twist()
    self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    self._takeoff_msg = Empty()
    self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
    self._land_msg = Empty()
    
    # make the drone takeoff
    i=0
    while not i == 3:
        self._pub_takeoff.publish(self._takeoff_msg)
        rospy.loginfo('Taking off...')
        time.sleep(1)
        i += 1
    
    # define the seconds to move in each side of the square (which is taken from the goal) and the seconds to turn
    sideSeconds = 2.0
    turnSeconds = 1.8
    
    self.move_drone((0,0,1.4))
    time.sleep(0.6)
    self.stop_drone()
    time.sleep(1000)
    
    i=0
    while not i == 3:
        self._pub_land.publish(self._land_msg)
        rospy.loginfo('Landing...')
        time.sleep(1)
        i += 1
      
if __name__ == '__main__':
  rospy.init_node('race')
  run_race = RunRace()
  try:
      run_race.run_race()
  except rospy.ROSInterruptException:
      pass