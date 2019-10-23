from time import sleep
import cv2
import cv2.cv as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def find_ball(data):
	cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
	
	

def main() :
	

