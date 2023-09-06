import rospy
import time

from geometry_msgs.msg import Vector3Stamped,Twist,PoseStamped
from sensor_msgs.msg import Joy, Image, CompressedImage

class delaynode(object):

    def __init__(self,delay):
        super().__init__()

        self.delay = delay
        self.cockpit_subscribe = rospy.Subscriber("/delay/racing_cockpit/ctrl_cmd",  Vector3Stamped, self.ctrl_callback, queue_size=10)
        self.cockpit_publisher = rospy.Publisher("/racing_cockpit/ctrl_cmd",  Vector3Stamped, queue_size=10)
        self.feedback_subscribe = rospy.Subscriber("/delay/adaptive_response",  Vector3Stamped, self.ff_callback, queue_size=10)
        self.feedback_publisher = rospy.Publisher("/adaptive_response",  Vector3Stamped, queue_size=10)

    def ctrl_callback(self,data):
        self.cockpit_publisher.publish(data)
        time.sleep(self.delay)
    
    def ff_callback(self,data):
        self.feedback_publisher.publish(data)
        time.sleep(self.delay)

    def shutdown(self):
        rospy.loginfo("Beginning shutdown routine...")
            rospy.loginfo("Shutting down cleanly...")
        
if __name__ == '__main__':
    rospy.init_node('delaynode')
    racing = delaynode(o.5)
    rospy.on_shutdown(racing.shutdown)

    try:
        # racing.publish_data()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')