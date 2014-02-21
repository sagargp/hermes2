#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

class JoyNode:
  def __init__(self):
    rospy.init_node('joy_teleop')
    self.joy = rospy.Subscriber('joy', Joy, self.joy_callback)

  def joy_callback(self, joy):
    #(a, b, x , y, lb, rb, back, start, guide, left_joy, right_joy, dpad_left, dpad_right, dpad_up, dpad_down) = joy.buttons
    #(left_x, left_y, left_trigger, right_x, right_y, right_trigger) = joy.axes
    (a, b, x , y, lb, rb, back, start, guide, left_joy, right_joy) = joy.buttons
    (left_x, left_y, left_trigger, right_x, right_y, right_trigger, dpad_x, dpad_y) = joy.axes

    left_x  = int(left_x * 255)
    left_y  = int(left_y * 255)
    right_x = int(right_x * 255)
    right_y = int(right_y * 255)

    #rospy.loginfo(("left_joy: ", left_joy))
    #rospy.loginfo(("right_joy: ", right_joy))
    #rospy.loginfo(("left_x: ", left_x ))
    #rospy.loginfo(("left_y: ", left_y))
    #rospy.loginfo(("left_trigger: ", left_trigger))
    #rospy.loginfo(("right_trigger: ", right_trigger))
    rospy.loginfo(("right_y: ", right_y))
    rospy.loginfo(("right_x: ", right_x))
    #rospy.loginfo(("dpad_x: ", dpad_x))
    #rospy.loginfo(("dpad_y: ", dpad_y))

def main():
  joy_node = JoyNode()
  rospy.loginfo('joystick controller starting')

  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

  rospy.loginfo('joystick controller ending')

if __name__ == '__main__':
  main()
