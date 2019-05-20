#!usr/bin/env python3
import RPi.GPIO as IO
import time
from simple_pid import PID 
import rospy
from sensor_msgs.msg import Imu
import asciichartpy as asc
import os
from pyquaternion import Quaternion
control_input = [0]
first = 1
direction = 1;

def callback(data):
	if first == 1:
		bearing = data.orientation.z
		pid.setpoint = bearing
		global first
		first=0
	clear()
	u=data.orientation.z
	#quat = Quaternion(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z)
	#unit_quat = quat.normalised
	print("control input: " +str(u))
	print("setpoint: " + str(pid.setpoint))
#	u_norm = (1-u)/2	
	#control_input.append(u)

	control = pid(u)
	control2 = pid1(u)
	#flip control signals if turning left is closer to the goal
	right.ChangeDutyCycle(control)
	left.ChangeDutyCycle(control)
#	print(asc.plot(control_input))
	print ("error: " + str(abs(u-pid.setpoint)))
#	forward()
	#calibrate()
	if (abs(u - pid.setpoint) < 0.05 ):
		forward()
		#backwards()
		#if direction == 1:
		#	forward()
		#	direction = 0
		#if (direction == 0):
		#	backwards()
		#	direction = 1

#	print(control_input)
#	print(u)
	print("right:" + str(control) + "   left:" + str(control2))
	
def forward():
	right.ChangeDutyCycle(6.5)
	left.ChangeDutyCycle(8.5)
def backwards():
	right.ChangeDutyCycle(8.5)
	left.ChangeDutyCycle(6.5)

def turn_right():
#	right.ChangeDutyCycle(right_stop)
	left.ChangeDutyCycle(7.5)
def spin_in_place():
	left.ChangeDutyCycle(7.5)
	right.ChangeDutyCycle(7.5)

def calibrate():
	left.ChangeDutyCycle(7.5)
	right.ChangeDutyCycle(7.5)

clear = lambda: os.system('clear')

setpoint = 0.59
left_stop = 7.1
right_stop = 7.08

left_backwards_max = 6.5
right_backwards_max = 8.5
left_forwards_max = 8.5
right_forwards_max = 6.5

#pid  = PID (4,1.5,0.3, setpoint)
pid  = PID (10,1.5,2, setpoint)
pid1 = PID(10,1.5,2, setpoint)

pid.output_limits = (6.5, 8.5)
pid1.output_limits = (6.5,8.5)
IO.setwarnings(False)
IO.setmode(IO.BCM)
IO.setup(13,IO.OUT)
IO.setup(19,IO.OUT)

right=IO.PWM(19,50)
left=IO.PWM(13,50)

right.start(0)
left.start(0)
#forward()
rospy.init_node('base_movement')
sub = rospy.Subscriber('/imu0',Imu,callback)
rospy.spin()
#while 1:
#	calibrate()
