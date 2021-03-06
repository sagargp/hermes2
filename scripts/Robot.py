import time
import argparse
import serial
import socket
import struct
import sys
import pickle

def get_voltage(s):
  s.write("VO")
  voltage = s.read(2) # read two bytes
  return ord(voltage[0]) * 256 + ord(voltage[1])

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

if __name__ == "__main__":
  argparser = argparse.ArgumentParser()
  argparser.add_argument('--tty', nargs=1, help='Serial port')
  argparser.add_argument('--baud', nargs=1, help='Serial baud rate')
  argparser.add_argument('--listen', nargs=1, help='IP:PORT of host that sends the joystick data')
  argparser.add_argument('--magic', nargs=1, help='Magic voltage divisor', default=68.319, type=float)
  args = argparser.parse_args(sys.argv[1:])

  serial_port      = args.tty[0]
  serial_baud      = args.baud[0]
  udp_ip, udp_port = args.listen[0].split(':')

  serial = serial.Serial(serial_port, serial_baud, timeout=1)
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind((udp_ip, int(udp_port)))

  speed_locked = False
  last_locked  = time.time()
  speed        = None
  turn         = None
  left_motor   = None
  right_motor  = None
  running      = True
  try:
    while running:
      data, addr = sock.recvfrom(1024)
      js_state = pickle.loads(data)

      if js_state['start_button']:
        serial.write(get_motor_packet(0, 0))
        running = False

      if js_state['start_button']:
        serial.write(get_motor_packet(0, 0))

      if js_state['left_stick_button'] and time.time() - last_locked > 0.5:
        speed_locked = not speed_locked
        last_locked = time.time()
        print "Toggling speed lock"

      if js_state['b_button']:
        voltage = get_voltage(serial)
        print "Voltage reads as: %d (%f)" % (voltage, voltage/args.magic)

      # Change mode to i2c. After this nothing will work any more!
      if js_state['back_button']:
        serial.write("CH")
        print "Changed mode."

      if not speed_locked:
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
  except:
    print "Killed"
  finally:
    serial.close()
