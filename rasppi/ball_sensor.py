import cv2
import numpy as np
import math
# width of the paper in inches
KNOWN_WIDTH = 7

#the focallength of the camera will need to be modified n  
#It is currently defaulted to my camera
#this number will vary due to different camera modules 

#laptop focallength 
#focalLength = 800
#webCam focallength
focalLength = 690

def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    return (knownWidth * focalLength) / perWidth

def get_distance(frame):
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv,(20,90,100), (30,255,255))
    blur = cv2.GaussianBlur(yellow, (5, 5), 0)
    edge = cv2.Canny(blur, 35, 125)
    contours, h = cv2.findContours(edge.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # find all contours
    max_area = -1
    max_box = None
    max_contour = None
    # loop through contours
    for c in contours:
        # rect is ((x, y), (w, h), rot)
        rect = cv2.minAreaRect(c)
        # gets the four points on the outside of the rectangle
        box = cv2.boxPoints(rect)
        # convert to int
        box = np.int0(box)
        # find area of contour
        area = cv2.contourArea(box)
        # if area is greater than max, set the max area and contour
        if area > max_area:
            max_area = area
            max_box = box
            max_contour = c
    angle = None
    hypotenuse = None
    #get the max countor detail
    x,y,w,h = cv2.boundingRect(max_box)
    #get image width
    imageW = frame.shape[1]
    #vertical distance
    distanceV = 0
    #horizontal Shift
    distanceH = 0
    # only draw the contour and calculate distance once the max_area has been set
    if max_area > -1:
        cv2.drawContours(frame, [max_box], 0, (0,255,0), 2)
        #this is the instant distance between paper and camera
        distanceV = distance_to_camera(KNOWN_WIDTH, focalLength, w)
        #default means when camera is at the origin of the paper
        default_W = (KNOWN_WIDTH * focalLength) / distanceV
        default_Shift = (imageW / 2) - (default_W / 2)
        #this is the instant distance of horizontal shift
        distanceH = (default_Shift - x) / focalLength * distanceV
        angle = math.atan(distanceH / distanceV) * 180 / math.pi
        hypotenuse = math.sqrt(distanceH * distanceH + distanceV * distanceV)
    return np.array([hypotenuse, angle, x,y,w,h])
