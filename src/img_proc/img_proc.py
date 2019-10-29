#!/usr/bin/env python
import rospy
from time import sleep
import cv2
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def imFill(im_in):
     
    # Copy the thresholded image.
    im_floodfill = im_in.copy()
     
    # Mask used to flood filling.
    # Notice the size needs to be 2 pixels than the image.
    h, w = im_in.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
     
    # Floodfill from point (0, 0)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
     
    # Invert floodfilled image
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
     
    # Combine the two images to get the foreground.
    im_out = im_in | im_floodfill_inv
    
    return im_out

def find_ball(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
    bgr_image = cv2.medianBlur(cv_image, 3)
    #bgr_image = cv_image

    # Convert input image to HSV
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    
    # Threshold the HSV image, keep only the green pixels
    green_hue_image = cv2.inRange(hsv_image, (40, 100, 100), (80, 255, 255))

    green_hue_image = cv2.GaussianBlur(green_hue_image, (9, 9), 2, 2)

    fill_image = imFill(green_hue_image)
    #fill_image = cv2.cvtColor(fill_image, cv2.COLOR_HSV2GRAY)
    print np.sum(fill_image > 0)
    ## Use the Hough transform to detect circles in the combined threshold image
    #circles = cv2.HoughCircles(fill_image, cv2.HOUGH_GRADIENT, 1.2, 1200, 100, 20, 10, 12);
    
    
    
    ## Loop over all detected circles and outline them on the original image
    #all_r = np.array([])
    #if circles is not None:
        #for i in circles[0]:

            #all_r = np.append(all_r, int(round(i[2])))
        #closest_ball = all_r.argmax()
        #center=(int(round(circles[0][closest_ball][0])), int(round(circles[0][closest_ball][1])))
        #radius=int(round(circles[0][closest_ball][2]))
        #cv2.circle(cv_image, center, radius, (0, 255, 0), 5);
    
    #cv2.namedWindow("Hue image", cv2.WINDOW_AUTOSIZE)
    #cv2.imshow("Hue image", hsv_image)
    #cv2.imshow("Circle image", fill_image)
    #cv2.waitKey(1)

def main() :

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('img_proc', anonymous=True)

    rospy.Subscriber("/usb_cam/image_raw", Image, find_ball)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()	

