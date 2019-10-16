#!/usr/bin/env python
import rospy
from picar import front_wheels, back_wheels
from picar.SunFounder_PCA9685 import Servo
import picar
from time import sleep
import numpy as np
from geometry_msgs.msg import Twist

bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()

picar.setup()

bw.speed = 0
fw.turn(90)
motor_speed = 20

curr_speed = 0
curr_dir = 0

def fw_movement(data):
    global curr_speed
    
    if data.linear.x > 0:
        curr_speed += 1
        if curr_speed > 5:
            curr_speed = 5
    elif data.linear.x < 0:
        curr_speed -= 1
        if curr_speed < -5:
            curr_speed = -5
    
    if curr_speed > 0:
        bw.speed = motor_speed * curr_speed
        bw.backward()
    elif curr_speed < 0:
        bw.speed = motor_speed * -curr_speed
        bw.forward()
    else:
        bw.stop()

def dir_movement(data):        
    global curr_dir
    
    if data.angular.z > 0:
        curr_dir += 1
        if curr_dir > 1:
            curr_dir = 1
    elif data.angular.z < 0:
        curr_dir -= 1
        if curr_dir < -1:
            curr_dir = -1
    
    if curr_dir > 0:
        fw.turn(0)
    elif curr_dir < 0:
        fw.turn(180)
    else:
        fw.turn(90) 
    
def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('car_control', anonymous=True)

    rospy.Subscriber("turtle1/cmd_vel", Twist, fw_movement)
    rospy.Subscriber("turtle1/cmd_vel", Twist, dir_movement)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()