def make_packet(packet):
  checksum = packet[0]
  for c in packet[1:]:
    checksum = checksum ^ c

  p2 = packet
  p2.append(checksum)
  return struct.pack("BBBBBBBB", *p2), checksum

if __name__ == "__main__":
  import argparse
  import json
  import serial
  import socket
  import struct
  import sys

  ID_START  = 255
  ID_END    = 254
  ID_MOTOR  = 98
  ID_CONFIG = 99

  argparser = argparse.ArgumentParser()
  argparser.add_argument('--tty', nargs=1, help='Serial port')
  argparser.add_argument('--baud', nargs=1, help='Serial baud rate')
  argparser.add_argument('--listen', nargs=1, help='IP address/port to listen on')
  argparser.add_argument('--debug', action='store_true', default=False, help='Show debug info')
  args = argparser.parse_args(sys.argv[1:])

  serial_port      = args.tty[0]
  serial_baud      = args.baud[0]
  udp_ip, udp_port = args.listen[0].split(':')

  serial = serial.Serial(serial_port, serial_baud, timeout=1)
  sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind((udp_ip, int(udp_port)))

  if args.debug:
    output, checksum = make_packet([ID_START, ID_CONFIG, 1, 0, 0, 0, ID_END])
    serial.write(output)

  while True:
    data, addr = sock.recvfrom(1024)
    commands = json.loads(data)

    for command in commands:
      # get the back/forth direction from the left joystick
      if command['joystick'] == 0:
        y = float(command['y'])

      # get the left/right direction from the left joystick
      if command['joystick'] == 1:
        x = float(command['x'])

    x_sign = 0
    y_sign = 0
    if x < 0: x_sign = 1
    if y < 0: y_sign = 1

    # change the range to 0-255 (uchars)
    x_byte = int(abs(x)/100*255)
    y_byte = int(abs(y)/100*255)

    motor_pk = [ID_START, ID_MOTOR, x_sign, x_byte, y_sign, y_byte, ID_END]
    output, checksum = make_packet(motor_pk)
    serial.write(output) 
    
    if args.debug:
      print "W: ", ','.join([str(x) for x in motor_pk])
      print "R: ", 
      read = serial.readline()
      if read.startswith("ret:"):
        s = read[4:-1]
        d = [str(ord(x)) for x in s]
        print ','.join(d)
      else:
        print read
