import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class LineFollower(object):

    def __init__(self) -> None:
        rospy.init_node('CameraFeed', anonymous=True, disable_signals=True)
        rospy.Subscriber("/drone/down_camera/image_raw", Image, self.down_camera_cb)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.lower_bound = np.array([72, 72, 72])
        self.upper_bound = np.array([90,90,90])

    def down_camera_cb(self, msg: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(msg)
        mask = cv2.inRange(img, self.lower_bound, self.upper_bound)
        cv2.imshow("stream", mask)
        cv2.waitKey(1)

def main():
    lineFollower = LineFollower()
    while not rospy.is_shutdown():
        rospy.spin()
            
if __name__ == '__main__':
    main()
