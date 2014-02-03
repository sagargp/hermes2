#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import sys
import serial
import struct
import time

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
    self.serial       = serial
    self.last_message = None
    self.last_motor   = None
    self.voltage_low  = False

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

      # kill motors if we haven't received a command in 2 seconds
      if self.last_message is not None and time.time() - self.last_message > 2.0 and not self.voltage_low:
        self.serial.write(get_motor_packet(0, 0))

      if self.last_motor is not None:
        self.serial.write(self.last_motor)

  def joy_callback(self, joy):
    self.last_message = time.time()

    (a, b, x , y, lb, rb, back, start, guide, left_joy, right_joy) = joy.buttons
    (left_x, left_y, right_x, right_y, right_trigger, left_trigger, dpad_x, dpad_y) = joy.axes

    left_y  = int(left_y * 255)
    right_x = int(right_x * 255)

    speed = -left_y
    turn = right_x

    left_motor  = int((speed+turn)*255)
    right_motor = int((speed-turn)*255)

    if left_motor > 255: left_motor = 255
    if left_motor < -255: left_motor = -255
    if right_motor > 255: right_motor = 255
    if right_motor < -255: right_motor = -255

    if abs(left_motor) < 90: left_motor = 0
    if abs(right_motor) < 90: right_motor = 0

    #self.last_motor = get_motor_packet(left_motor, right_motor)
    self.serial.write(get_motor_packet(left_motor, right_motor))

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
