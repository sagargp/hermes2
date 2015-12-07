#!/usr/bin/env python

# interfaces over serial to the arduino board
# * subscribes to Twist messages and converts them to motor commands
# * reads voltage and publishes "voltage" messages

import serial 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    
def listener():
    rospy.init_node("hermes-platform", anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
