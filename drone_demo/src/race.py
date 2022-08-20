#! /usr/bin/env python3

from cv2 import computeCorrespondEpilines
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
    self.down_camera_crop_ratio = 0.8
    self.down_camera_yaw_k = -1.1 # yaw p controller multiplier, keep line vertical
    self.down_camera_y_k = 0.001 # y p controller multiplier, keep line centered
    self.forward_speed = 0.3
    self.upward_speed = 0.3
    self.front_camera_gate_z_k = -0.005 # -0.0008
    self.front_camera_yaw_k = -0.0008
    self.gate_lower_bound = np.array([235, 62, 150])
    self.gate_upper_bound = np.array([275, 94, 179])
    self.gate_align_time = 6.8
    
  
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
  
  def get_cropped_img(self, img, ratio):
    height, width, _ = img.shape
    h = int(height * self.down_camera_crop_ratio)
    w = int(width * self.down_camera_crop_ratio)
    h_center = height // 2
    w_center = width // 2
    return img[h_center - h // 2: h_center + h // 2, w_center - w // 2: w_center + w // 2]


  def down_camera_cb(self, msg):
    if self.state == "LINEFOLLOW":
      img = self.bridge.imgmsg_to_cv2(msg)
      height, width, _ = img.shape

      # compute slope of line perpendicular to line on the ground
      img_cropped = self.get_cropped_img(img, self.down_camera_crop_ratio)
      crop_height, crop_width, _ = img_cropped.shape
      mask = cv2.inRange(img_cropped, self.line_lower_bound, self.line_upper_bound)
      edges = cv2.Canny(mask, 75, 150)
      lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap = 50)
      if lines is not None:
        x1, y1, x2, y2 = lines[0][0]
        slope = self.compute_slope(x1, y1, x2, y2)
        perp_slope = -1 / slope
        horiz_offset = crop_width // 2 - (x2 + x1) // 2
        cv2.line(img_cropped, (crop_width // 2, crop_height // 2), ((x2 + x1) // 2, crop_height // 2), (0, 255, 0), 5)
        cv2.line(img_cropped, (x1, y1), (x2, y2), (0, 0, 255), 5)
        # cv2.imshow("stream", img_cropped)
        # cv2.waitKey(1)
      else:
        print("Could not find line, zooming out")
        img = self.bridge.imgmsg_to_cv2(msg)
        mask = cv2.inRange(img, self.line_lower_bound, self.line_upper_bound)
        edges = cv2.Canny(mask, 75, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap = 50)
        if lines is not None:
          x1, y1, x2, y2 = lines[0][0]
          slope = self.compute_slope(x1, y1, x2, y2)
          perp_slope = -1 / slope
          horiz_offset = width // 2 - (x2 + x1) // 2
          cv2.line(img, (width // 2, height // 2), ((x2 + x1) // 2, height // 2), (0, 255, 0), 5)
          cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
          # cv2.imshow("stream", img)
          # cv2.waitKey(1)
        else:
          perp_slope = np.random.randint(-5, 5)
          horiz_offset = np.random.choice([-1, 1])

      # target is for the normal line slope to be 0 - employ P controller for this
      self.x = self.forward_speed
      self.y = self.down_camera_y_k * horiz_offset
      self.yaw = self.down_camera_yaw_k * perp_slope
    
    # if self.state == "FORWARD":
    #   img = self.bridge.imgmsg_to_cv2(msg)
    #   height, width, _ = img.shape
    #   img_cropped = self.get_cropped_img(img, 0.1)
    #   mask = cv2.inRange(img_cropped, self.line_lower_bound, self.line_upper_bound)
    #   cv2.imshow("stream", mask)
    #   cv2.waitKey(1)
    #   self.x = self.forward_speed
    #   self.y = 0
    #   self.yaw = 0
    #   # self.state = "HOVER"
  
  def forward(self):
    self.turn_drone(0.0)
    self.move_drone((0.3,0,0))
    time.sleep(5.5)
    self.move_drone((0,0,0))
    self.state = "HOVER"

  def takeoff(self):
    # make the drone takeoff
    i = 0
    self.x = 0
    self.y = 0
    self.z = 0
    self.yaw = 0
    while not i == 3:
        self._pub_takeoff.publish(self._takeoff_msg)
        rospy.loginfo('Taking off...')
        time.sleep(1)
        i += 1
    # self.move_drone((0,0,1.2))
    # time.sleep(0.54)
    # self.move_drone((0,0,-1))
    # time.sleep(0.1)
    # self.move_drone((0,0,0))
    # time.sleep(0.2)
    self.start_gate_align = time.time()
    self.state = "GATE_ALIGN"
    
  def land(self):
    i=0
    while not i == 3:
        self._pub_land.publish(self._land_msg)
        rospy.loginfo('Landing...')
        time.sleep(1)
        i += 1
    self.state = "LAND"
  
  def compute_slope(self, x1, y1, x2, y2):
    if abs(x2 - x1) < 0.1:
      return 90
    else:
      return (y2 - y1) / (x2 - x1)
  
  def front_camera_cb(self, msg: Image) -> None:
    img = self.bridge.imgmsg_to_cv2(msg)
    if self.state == "LINEFOLLOW":
      #img_cropped = img[:img.shape[0]//2]
      gate_line_ys = []
      mask = cv2.inRange(img, self.gate_lower_bound, self.gate_upper_bound)
      img_edges = cv2.Canny(mask, 40, 120) # 50 150 previously
      lines = cv2.HoughLinesP(img_edges, 1, np.pi / 180, 50, maxLineGap = 50)
      # if lines is not None:
      #   for line in lines:
      #     x1, y1, x2, y2 = line[0]
      #     cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)

      if lines is not None:
        for line in lines:
          x1, y1, x2, y2 = line[0]
          if (abs(y2 - y1)) < 40:
            continue
          slope = (x1 - x2) / (y2 - y1)
          if abs(slope) < 1.0 and len(gate_line_ys) == 0:
            gate_line_ys.append((y1 + y2) / 2)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
            break
          # elif abs(slope) < 1.0 and abs(y1 - gate_line_ys[0]) > 70:
          #   print("Found another line 70 apart")
          #   gate_line_ys.append(y1)
          #   cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
          #   break
        cv2.imshow("stream", img)
        cv2.waitKey(1)
        if len(gate_line_ys) > 0:
          self.z = (gate_line_ys[0] - img.shape[0] / 2) * self.front_camera_gate_z_k
        else:
          self.z = 0
      else:
        self.z = 0
    mask = 0

    if self.state == "GATE_ALIGN":
      gate_line_xs = []
      gate_line_ys = []
      mask = cv2.inRange(img, self.gate_lower_bound, self.gate_upper_bound)
      img_edges = cv2.Canny(mask,40, 120)
      lines = cv2.HoughLinesP(img_edges, 1, np.pi / 180, 50, maxLineGap = 50)
      if lines is not None:
        for line in lines:
          x1, y1, x2, y2 = line[0]
          if (abs(y1 - y2)) < 40:
            continue
          slope = (x2 - x1) / (y1 - y2)
          if abs(slope) < 1.0 and len(gate_line_xs) == 0:
            gate_line_xs.append(x1)
            gate_line_ys.append((y1 + y2) / 2)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
          elif abs(slope) < 1.0 and abs(x1 - gate_line_xs[0]) > 40:
            gate_line_xs.append(x1)
            gate_line_ys.append((y1 + y2) / 2)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
            break
        cv2.imshow("stream", img)
        cv2.waitKey(1)
        self.x = self.forward_speed
        self.y = 0
        if len(gate_line_ys) > 0:
          self.z = max(min((gate_line_ys[0] - img.shape[0] / 2) * self.front_camera_gate_z_k, self.upward_speed), -self.upward_speed)
        else:
          self.z = 0
        if len(gate_line_xs) > 1:
          # delta_h = (gate_line_ys[0] + gate_line_ys[1]) / 2 - img.shape[0] / 2
          # self.z = max(min(delta_h * self.front_camera_z_k, self.upward_speed), -self.upward_speed)
          self.yaw = ((gate_line_xs[0] + gate_line_xs[1]) / 2 - img.shape[1] / 2) * self.front_camera_yaw_k
        else:
          self.yaw = 0
      else:
        #cv2.imshow(bogus)
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = -0.3
      if time.time() - self.start_gate_align > self.gate_align_time:
        print("Switching to LINEFOLLOW mode")
        self.state = "LINEFOLLOW"
    
    """cv2.imshow("stream", mask)
    cv2.waitKey(1)"""
    

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
      elif self.state == "LAND":
        self.land()
      self.move_publish()
      #print(self.state)
if __name__ == '__main__':
  run_race = RunRace()
  try:
      run_race.run_race()
  except rospy.ROSInterruptException:
      pass