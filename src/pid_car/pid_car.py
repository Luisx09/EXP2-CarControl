#!/usr/bin/env python
import rospy
from picar import front_wheels, back_wheels
from picar.SunFounder_PCA9685 import Servo
import picar
from time import sleep
import numpy as np
from std_msgs.msg import Float64

bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
pan_servo = Servo.Servo(1)
tilt_servo = Servo.Servo(2)
picar.setup()

bw.speed = 0
fw.turn(90)
pan_servo.write(90)
tilt_servo.write(90)
motor_speed = 100

curr_speed = 0
curr_dir = 0

def fw_movement(data):
    curr_speed = data.data
    
    if curr_speed > 0:
        bw.speed = int(round(curr_speed))
        bw.backward()
    elif curr_speed < 0:
        bw.speed = int(round(-curr_speed))
        bw.forward()
    else:
        bw.stop()
    
def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pid_car', anonymous=True)

    rospy.Subscriber("control_spd", Float64, fw_movement)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
