import rospy
import time
from std_msgs.msg import Empty

class Land(object):

    def __init__(self):
        rospy.init_node('Land', anonymous=True, disable_signals=True)
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self.rate = rospy.Rate(10)

    def land(self):
        i=0
        while not i == 3:
            empty = Empty()
            self._pub_land.publish(empty)
            rospy.loginfo('Landing...')
            time.sleep(1)
            i += 1

if __name__ == '__main__':
  land = Land()
  land.land()