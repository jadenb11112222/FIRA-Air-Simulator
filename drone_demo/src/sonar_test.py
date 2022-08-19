import rospy
import time
from sensor_msgs.msg import Range

class SonarTest(object):

    def __init__(self):
        rospy.init_node('SonarTest', anonymous=True, disable_signals=True)
        rospy.Subscriber('/drone/sonar', Range, self.sonar_cb)
        self.rate = rospy.Rate(10)

    def sonar_cb(self, msg: Range) -> None:
        rospy.loginfo(msg)

if __name__ == '__main__':
    sonarTest = SonarTest()
    while not rospy.is_shutdown():
        rospy.spin()