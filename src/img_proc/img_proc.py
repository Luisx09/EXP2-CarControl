#!/usr/bin/env python
import rospy
from time import sleep
import cv2
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
prev_dist = 0.0
agg_dist = 0
valid_counts = 0
counter = 0

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
    global pub_dist, agg_dist, valid_counts, counter
    # Get current parameter values and store them
    color_num = rospy.get_param('~color', 60)
    sensitivity = rospy.get_param('~sensitivity', 20)
    samples = rospy.get_param('~samples', 10)
    
    cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
    bgr_image = cv2.medianBlur(cv_image, 3)
    #bgr_image = cv_image

    # Convert input image to HSV
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    
    # Threshold the HSV image, keep only the green pixels
    green_hue_image = cv2.inRange(hsv_image, (color_num - sensitivity, 100, 100), (color_num + sensitivity, 255, 255))
    green_hue_image = cv2.GaussianBlur(green_hue_image, (9, 9), 2, 2)

    fill_image = imFill(green_hue_image)
    white_pix = np.sum(fill_image > 0)
    #fill_image = cv2.cvtColor(fill_image, cv2.COLOR_HSV2GRAY)
    rospy.loginfo('White Pixels: '+str(white_pix))

    #calc_dist = np.polyval(poly_dist, white_pix)
    if white_pix != 0:
        calc_dist = 3625.15 * (white_pix ** -0.56)
        agg_dist += calc_dist
        valid_counts += 1
        rospy.loginfo('Distance to Ball: '+str(round(calc_dist, 4)))
    if valid_counts != 0:
        mean_dist = agg_dist / valid_counts
        pub_dist.publish(mean_dist)
    if valid_counts >= samples:
        agg_dist -= mean_dist
        valid_counts -= 1

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

    global pub_dist
    rospy.init_node('img_proc', anonymous=True)
    #rospy.get_param('color', 60) # Green
    #rospy.get_param('sensitivity', 20)
    pub_dist = rospy.Publisher('/distance', Float64, queue_size=10)

    rospy.Subscriber("/usb_cam/image_raw", Image, find_ball)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

