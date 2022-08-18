#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraFeed(object):

    def __init__(self):
        rospy.init_node('CameraFeed', anonymous=True, disable_signals=True)
        rospy.Subscriber("/drone/front_camera/image_raw", Image, self.front_camera_cb)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)

    def front_camera_cb(self, msg: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        cv2.imshow("stream", img)
        cv2.waitKey(1)

def main():
    cameraFeed = CameraFeed()
    while not rospy.is_shutdown():
        rospy.spin()
            
if __name__ == '__main__':
    main()