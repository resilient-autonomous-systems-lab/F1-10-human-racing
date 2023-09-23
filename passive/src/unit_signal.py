#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

def joy_publisher():
    # Initialize the ROS node
    rospy.init_node('joy_publisher', anonymous=True)

    # Create a publisher for the /G29/joy topic
    pub = rospy.Publisher('/G29/joy', Joy, queue_size=10)

    # Create a Joy message
    joy_msg = Joy()
    
    # Initialize the axes field with a unit step signal for axes[1]
    joy_msg.axes = [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]
    joy_msg.buttons = [0] * 20  # 11 buttons in total
    joy_msg.buttons[12] = 1

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Publish the Joy message
        pub.publish(joy_msg)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        joy_publisher()
    except rospy.ROSInterruptException:
        pass

