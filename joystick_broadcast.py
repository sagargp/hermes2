import pygame
import socket
import argparse
import pickle
import sys

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
  argparser.add_argument('--server', nargs=1, help='IP address/port to listen on')
  args = argparser.parse_args(sys.argv[1:])

  udp_ip, udp_port = args.server[0].split(':')
  udp_port = int(udp_port)
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  pygame.init()
  joystick = pygame.joystick.Joystick(0)
  joystick.init()

  running = True
  while running:
    pygame.event.wait()

    js_data = get_xbox_state(joystick)
    sock.sendto(pickle.dumps(js_data), (udp_ip, udp_port))

    if js_data['start_button']:
      running = False
