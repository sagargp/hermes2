#!/usr/bin/env python

# interfaces over serial to the arduino board
# * subscribes to Twist messages and converts them to motor commands
# * reads voltage and publishes "voltage" messages
import serial 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

motor_driver = None

def scale(value, _from, _to):
    _min, _max = _from
    _a, _b = _to
    return (_b - _a)*(value-_min)/(_max-_min) + _a

class TestSerial(object):
    def __init__(self, port=None, baudrate=9600, **kwargs):
        print("Created a fake serial port at %s with baud rate %d" % (port, baudrate))

    def write(self, message):
        print("Serial writing: %s" % message)

class Motors(object):
    def __init__(self, serial_dev, robot_width=0.254):
        self._serial = serial_dev
        self.width = robot_width

    def drive(self, angular, linear):
        v_left   = linear.x
        v_right  = linear.x
        v_left  += angular.z * -self.width/2.0
        v_right += angular.z * self.width/2.0

        # assume max PWM -> 10m/s
        # cap the velocities at that max
        if v_left > 10.0: v_left = 10.0
        if v_left < -10.0: v_left = -10.0
        if v_right > 10.0: v_right = 10.0
        if v_right < -10.0: v_right = -10.0

        left_pwm = scale(v_left,   (-10.0, 10.0), (0, 255))
        right_pwm = scale(v_right, (-10.0, 10.0), (0, 255))

        self._serial.write("HB %d %d" % (left_pwm, right_pwm))

def callback(data):
    if motor_driver is None:
        return

    angular = data.angular
    linear  = data.linear
    motor_driver.drive(angular, linear)
    
if __name__ == '__main__':
    rospy.init_node("hermes_platform", anonymous=True)

    robot_width = rospy.get_param('/robot_width', 0.254)

    # set up the serial port
    serial_port = rospy.get_param('/serial_port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('/serial_port', 9600)
    # serial_dev = serial.Serial(serial_port, serial_baud)
    serial_dev = TestSerial(serial_port, serial_baud)
    motor_driver = Motors(serial_dev, robot_width)

    # subscribe to Twist messages from the joystick
    rospy.Subscriber("cmd_vel", Twist, callback)

    # run forever
    rospy.spin()
