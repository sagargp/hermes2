import argparse
import json
import serial
import socket
import struct
import sys
import pygame

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

  return struct.pack('BBBBBB', ord('H'), ord('B'), left_mode, left_command, right_mode, right_command)

def get_xbox_state(joystick):
  joystick_state = {}
  joystick_state['a_button']           = joystick.get_button(0)
  joystick_state['b_button']           = joystick.get_button(1)
  joystick_state['x_button']           = joystick.get_button(2)
  joystick_state['y_button']           = joystick.get_button(3)
  joystick_state['left_button']        = joystick.get_button(4)
  joystick_state['right_button']       = joystick.get_button(5)
  joystick_state['back_button']        = joystick.get_button(6)
  joystick_state['start_button']       = joystick.get_button(7)
  joystick_state['center_button']      = joystick.get_button(8)
  joystick_state['left_stick_button']  = joystick.get_button(9)
  joystick_state['right_stick_button'] = joystick.get_button(10)
  joystick_state['left_axis_x']        = joystick.get_axis(0)
  joystick_state['left_axis_y']        = joystick.get_axis(1)
  joystick_state['right_axis_x']       = joystick.get_axis(2)
  joystick_state['right_axis_y']       = joystick.get_axis(3)
  joystick_state['right_trigger']      = joystick.get_axis(4)
  joystick_state['left_trigger']       = joystick.get_axis(5)
  return joystick_state

if __name__ == "__main__":
  argparser = argparse.ArgumentParser()
  argparser.add_argument('--tty', nargs=1, help='Serial port')
  argparser.add_argument('--baud', nargs=1, help='Serial baud rate')
  args = argparser.parse_args(sys.argv[1:])

  serial_port      = args.tty[0]
  serial_baud      = args.baud[0]

  serial = serial.Serial(serial_port, serial_baud, timeout=1)

  pygame.init()
  joystick = pygame.joystick.Joystick(0)
  joystick.init()

  while True:
    pygame.event.pump()
    js_state = get_xbox_state(joystick)

    speed = -js_state['left_axis_y']
    turn = js_state['right_axis_x']

    left_motor  = int((speed + turn) * 255)
    right_motor = int((speed - turn) * 255)

    if left_motor > 255: left_motor = 255
    if left_motor < -255: left_motor = -255
    if right_motor > 255: right_motor = 255
    if right_motor < -255: right_motor = -255

    if abs(left_motor) < 90: left_motor = 0
    if abs(right_motor) < 90: right_motor = 0

    serial.write(get_motor_packet(left_motor, right_motor))
