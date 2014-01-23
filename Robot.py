import argparse
import json
import serial
import socket
import struct
import sys

def get_motor_packet(left_speed, right_speed):
  """
  Speeds should be between -255 and 255
  """
  if left_speed < 0:
    left_mode = 0
    left_command = -left_speed
  elif left_speed > 0:
    left_mode = 2
    left_command = left_speed
  else:
    left_mode = 1
    left_command = 0

  if right_speed < 0:
    right_mode = 0
    right_command = -right_speed
  elif right_speed > 0:
    right_mode = 2
    right_command = right_speed
  else:
    right_mode = 1
    right_command = 0

  return struct.pack('BBBBBB', ord('H'), ord('B'), left_mode, left_command, right_mode, right_command)

if __name__ == "__main__":
  argparser = argparse.ArgumentParser()
  argparser.add_argument('--tty', nargs=1, help='Serial port')
  argparser.add_argument('--baud', nargs=1, help='Serial baud rate')
  args = argparser.parse_args(sys.argv[1:])

  serial_port      = args.tty[0]
  serial_baud      = args.baud[0]

  serial = serial.Serial(serial_port, serial_baud, timeout=1)

  while True:
    pass
