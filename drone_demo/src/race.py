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
    rospy.init_node('RunRace', anonymous=True, disable_signals=True)
    self.state = "TAKEOFF"
    self.ctrl_c = False
    self.rate = rospy.Rate(10)
    self.x = 0.0
    self.y = 0.0
    self.z = 0.0
    self.yaw = 0.0

    # define the different publishers, subscribers, and messages that will be used]
    rospy.Subscriber("/drone/down_camera/image_raw", Image, self.down_camera_cb)
    rospy.Subscriber("/drone/front_camera/image_raw", Image, self.front_camera_cb)
    self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self._move_msg = Twist()
    self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    self._takeoff_msg = Empty()
    self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
    self._land_msg = Empty()
    self.bridge = CvBridge()
    self.rate = rospy.Rate(10)
    self.line_lower_bound = np.array([72, 72, 72])
    self.line_upper_bound = np.array([90, 90, 90])
    self.down_camera_crop_ratio = 5 # get rid of 1/x around the border when cropping
    self.down_camera_yaw_k = -1.1 # yaw p controller multiplier, keep line vertical
    self.down_camera_y_k = 0.2 # y p controller multiplier, keep line centered
    self.forward_speed = 0.3
    self.front_camera_k = -0.0005
    self.front_camera_yaw = -0.0005
    self.gate_lower_bound = np.array([250, 72, 160])
    self.gate_upper_bound = np.array([260, 84, 169])
    
  
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

  def gate_alignment(self):
    return

  def down_camera_cb(self, msg):
    if self.state != "LINEFOLLOW": return
    img = self.bridge.imgmsg_to_cv2(msg)
    width, height, _ = img.shape

    # compute slope of line perpendicular to line on the ground
    crop_width_start = int(width / self.down_camera_crop_ratio)
    crop_width_end = width - crop_width_start
    crop_height_start = int(height / self.down_camera_crop_ratio)
    crop_height_end = height - crop_height_start
    img_cropped = img[crop_width_start: crop_width_end, crop_height_start: crop_height_end]
    mask = cv2.inRange(img_cropped, self.line_lower_bound, self.line_upper_bound)
    edges = cv2.Canny(mask, 75, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap = 50)
    if lines is not None:
      x1, y1, x2, y2 = lines[0][0]
      slope = (y2 - y1) / (x2 - x1)
      perp_slope = -1 / slope
      horiz_offset = img_cropped.shape[0] / 2 - (x2 + x1 / 2)
      cv2.line(img_cropped, (x1, y1), (x2, y2), (0, 0, 255), 5)
      print(f"slope: {slope}, perp slope: {perp_slope}")
      #cv2.imshow("stream", img_cropped)
      #cv2.waitKey(1)
    else:
      print("Could not find line, zooming out")
      img = self.bridge.imgmsg_to_cv2(msg)
      mask = cv2.inRange(img, self.line_lower_bound, self.line_upper_bound)
      edges = cv2.Canny(mask, 75, 150)
      lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap = 50)
      if lines is not None:
        x1, y1, x2, y2 = lines[0][0]
        slope = (y2 - y1) / (x2 - x1)
        perp_slope = -1 / slope
        horiz_offset = img.shape[0] / 2 - (x2 + x1 / 2)
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
        #cv2.imshow("stream", img)
        #cv2.waitKey(1)
      else:
        perp_slope = np.random.randint(-5, 5)
        horiz_offset = np.random.choice([-1, 1])

    print(f"horiz offset: {horiz_offset}, perp slope: {perp_slope}")
    # target is for the normal line slope to be 0 - employ P controller for this
    self.x = self.forward_speed
    self.y = 0 # self.down_camera_y_k * horiz_offset
    self.z = 0
    self.yaw = self.down_camera_yaw_k * perp_slope

  def takeoff(self):
    # make the drone takeoff
    i = 0
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
    self.state = "GATE_ROTATE"
    
  def forward(self):
    self.turn_drone(-0.1)
    time.sleep(2.8)
    self.turn_drone(0)
    time.sleep(0.5)
    self.move_drone((0.3,0,0))
    time.sleep(5.5)
    self.move_drone((0,0,0))
    self.state = "HOVER"

  def land(self):
    i=0
    while not i == 3:
        self._pub_land.publish(self._land_msg)
        rospy.loginfo('Landing...')
        time.sleep(1)
        i += 1
    self.state = "LAND"
  
  def front_camera_cb(self, msg: Image) -> None:
    img = self.bridge.imgmsg_to_cv2(msg)
    if self.state == "GATE_ALIGNMENT":
      #img_cropped = img[:img.shape[0]//2]
      gate_lines = []
      mask = cv2.inRange(img, self.gate_lower_bound, self.gate_upper_bound)
      img_edges = cv2.Canny(mask, 50, 150)
      lines = cv2.HoughLinesP(img_edges, 1, np.pi / 180, 50, maxLineGap = 50)
      if lines is not None:
        count = 0
        previous_line_y = lines[0][0][1]
        for line in lines:
          x1, y1, x2, y2 = line[0]
          slope = (y2 - y1) / (x2 - x1)
          if abs(slope) < 1.0 and len(gate_lines) == 0:
            gate_lines.append(line)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
          elif abs(slope) < 1.0 and abs(y1 - gate_lines[0][0][1]) > 10:
            gate_lines.append(line)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
            break
        #cv2.imshow("stream", img)
        #cv2.waitKey(1)
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.z = ((gate_lines[0][0][1] + gate_lines[1][0][1])/2 - img.shape[0]/2) * self.front_camera_k
      else:
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0.04
    
    if self.state == "GATE_ROTATE":
      gate_lines = []
      mask = cv2.inRange(img, self.gate_lower_bound, self.gate_upper_bound)
      img_edges = cv2.Canny(mask, 50, 150)
      lines = cv2.HoughLinesP(img_edges, 1, np.pi / 180, 50, maxLineGap = 50)
      if lines is not None:
        count = 0
        previous_line_x = lines[0][0][0]
        for line in lines:
          x1, y1, x2, y2 = line[0]
          slope = (x2 - x1) / (y1 - y2)
          if abs(slope) < 1.0 and len(gate_lines) == 0:
            gate_lines.append(line)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
          elif abs(slope) < 1.0 and abs(y1 - gate_lines[0][0][1]) > 10:
            gate_lines.append(line)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
            break
        #cv2.imshow("stream", img)
        #cv2.waitKey(1)
        self.x = 0
        self.y = 0
        self.z = 0
        print(img.shape[1]/2)
        print(((gate_lines[0][0][0] + gate_lines[1][0][0])/2 - img.shape[1]/2))
        self.yaw = ((gate_lines[0][0][0] + gate_lines[1][0][0])/2 - img.shape[1]/2) * self.front_camera_yaw
        if ((gate_lines[0][0][0] + gate_lines[1][0][0])/2 - img.shape[1]/2) < 15.0:
          self.state = "FORWARD"
      else:
        #cv2.imshow(bogus)
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0.04
    

  def move_publish(self):
    self._move_msg.linear.x = self.x
    self._move_msg.linear.y = self.y
    self._move_msg.linear.z = self.z
    self._move_msg.angular.z = self.yaw
    self._pub_cmd_vel.publish(self._move_msg)

  def run_race(self):
    while(True):
      if self.state == "TAKEOFF":
        self.takeoff()
      elif self.state == "FORWARD":
        self.forward()
      elif self.state == "HOVER":
        self.move_drone((0,0,0))
        self.state = "LINEFOLLOW"
      elif self.state == "GATE_ALIGNMENT":
        self.gate_alignment()
      elif self.state == "LAND":
        self.land()
      self.move_publish()
      print(self.state)
if __name__ == '__main__':
  run_race = RunRace()
  try:
      run_race.run_race()
  except rospy.ROSInterruptException:
      pass