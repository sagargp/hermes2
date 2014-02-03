import smbus
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

  return struct.pack('BBBBBB', ord('H'), ord('B'), left_mode, left_command, right_mode, right_command)

bus = smbus.SMBus(1)
address = 0X04

bus.write_byte(address, ord('V'))
bus.write_byte(address, ord('O'))

response_0 = bus.read_byte(address)
response_1 = bus.read_byte(address)

print response_0 * 256 + response_1

for byte in get_motor_packet(100, 100):
  bus.write_byte(address, ord(byte))

time.sleep(2)

for byte in get_motor_packet(0, 0):
  bus.write_byte(address, ord(byte))
