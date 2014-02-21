#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import sys
import serial
import struct
import time
import threading

def get_motor_packet(left_speed, right_speed):
  """
  Speeds should be between -255 and 255
  """
  if left_speed < 0:
    left_mode = 2
    left_command = -left_speed
  elif left_speed > 0:
    left_mode = 0
    left_command = left_speed
  else:
    left_mode = 1
    left_command = 0

  if right_speed < 0:
    right_mode = 2
    right_command = -right_speed 
  elif right_speed > 0:
    right_mode = 0
    right_command = right_speed
  else:
    right_mode = 1
    right_command = 0

  return struct.pack('BBBBBB', 
    ord('H'), ord('B'), 
    left_mode, left_command,
    right_mode, right_command
  )

class Hermes:
  def __init__(self, serial):
    rospy.init_node('hermes2')
    self.joy = rospy.Subscriber('joy', Joy, self.joy_callback)
    self.serial = serial
    self.last_message_timestamp = None
    self.voltage_low = False
    self._speed = None
    self._turn = None

  def run(self):
    last_print = time.time()
    while not rospy.is_shutdown():
      if time.time() - last_print >= 1.0:
        last_print = time.time()

        # get the voltage
        num = self.serial.write("VO")
        voltage = self.serial.read(2)

        if len(voltage) == 2:
          voltage = ord(voltage[0]) * 256 + ord(voltage[1])
          voltage = voltage/68.319
          if voltage < 6.5:
            rospy.logwarn("Battery voltage is too low! %.2f V" % voltage)
            voltage_low = True
          else:
            rospy.loginfo("Battery voltage is %.2f V" % voltage)
            voltage_low = False
        else:
          voltage = 0.0
          rospy.logwarn("Reading voltage didn't work!")

      if self._speed is not None and self._turn is not None:
        lef_motor = int((self._speed + self._turn)*255)
        rig_motor = int((self._speed - self._turn)*255)

        if lef_motor >  255: lef_motor = 255
        if lef_motor < -255: lef_motor = -255
        if rig_motor >  255: rig_motor = 255
        if rig_motor < -255: rig_motor = -255

        if abs(lef_motor) < 90: lef_motor = 0
        if abs(rig_motor) < 90: rig_motor = 0

        self.serial.write(get_motor_packet(lef_motor, rig_motor))

      # kill motors if we haven't received a command in 2 seconds
      if self.last_message_timestamp is not None and time.time() - self.last_message_timestamp > 2.0 and not self.voltage_low:
        rospy.logwarn("Killing motors because I didn't get a command in 2 seconds")
        self.serial.write(get_motor_packet(0, 0))

  def joy_callback(self, joy):
    #(a, b, x , y, lb, rb, back, start, guide, left_joy, right_joy, dpad_left, dpad_right, dpad_up, dpad_down) = joy.buttons
    #(left_x, left_y, left_trigger, right_x, right_y, right_trigger) = joy.axes

    #(a, b, x , y, lb, rb, back, start, guide, left_joy, right_joy) = joy.buttons
    #(left_x, left_y, left_trigger, right_x, right_y, right_trigger, dpad_x, dpad_y) = joy.axes

    self.last_message_timestamp = time.time()
    self._speed = -joy.axes[1]
    self._turn = joy.axes[3]

if __name__ == "__main__":
  serial_port = "/dev/ttyUSB0"
  serial_baud = 9600
  serial = serial.Serial(serial_port, serial_baud, timeout=1)

  hermes = Hermes(serial)
  try:
    rospy.loginfo("Starting up")
    hermes.run()
  except rospy.ROSInterruptException:
    pass
  finally:
    serial.close()
