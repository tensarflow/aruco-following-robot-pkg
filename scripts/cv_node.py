#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#

import rospy
from std_msgs.msg import UInt16
import cv2
import cv2.aruco as aruco
import time
import matplotlib.pyplot as plt

def get_distance(ret,frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    shape_center_x = 0
    shape_center_y = 0
    dX = 0
    dY = 0

    if len(corners) > 0:
        for i in corners:
            for j in i:
                for k in j:
                    shape_center_x += k[0]
                    shape_center_y += k[1]

        shape_center_x = shape_center_x / 4
        shape_center_y = shape_center_y / 4

        height, width = gray.shape [:2]
        midX = width / 2
        midY = height / 2
        dX = shape_center_x - midX
        dY = shape_center_y - midY

    gray = aruco.drawDetectedMarkers(frame, corners)

    cv2.imshow('frame_',gray)
    cv2.waitKey(1)

    # return [dX,dY]
    return dY

def distanceGenerator():

    # Calculate number of pixels corresponding to 1 degree
    cap = cv2.VideoCapture(0)
    ret,frame = cap.read()
    height, width = frame.shape [:2]
    oneDeg = width/62


    # Initialize ROS environment
    pubServo = rospy.Publisher('servo_y', UInt16, queue_size=1)
    rospy.init_node('distanceGenerator', anonymous=True)
    rate = rospy.Rate(10) # wait untill it turns (in hz)

    initialPosition_y = 90
    actualPosition_y = initialPosition_y
    goneToInitial = False

    while not rospy.is_shutdown():

        # Go to initial position
        connections_x = pubServo.get_num_connections()
        rospy.loginfo('Connections X: %d', connections_x)

        if connections_x > 0 and goneToInitial == False:
            rospy.loginfo("Go to initial position")
            goneToInitial = True
            pubServo.publish(initialPosition_y)

        # Go to aruco position
        elif (connections_x > 0) and (goneToInitial == True):
            rospy.loginfo("Go to aruco position")

            # Get position of aruco marker
            ret,frame = cap.read()
            dY = get_distance(ret, frame)
            diffDeg_y = int(dY/oneDeg)
            arucoPosition_y = initialPosition_y + diffDeg_y

            # Control loop with arucoPosition as input and actualPosition as output
            TOL = 2 # Tolerance for controller
            while (abs(diffDeg_y) > TOL):

                actualPosition_y = actualPosition_y - diffDeg_y/5
                print "xxxxxxxxxxxxxxxxxxxxxxxxxxxxx: " + str(dY)

                if actualPosition_y < 0:
                    print("Can't go further...")

                elif actualPosition_y > 180.0:
                    print("Can't go further...")

                else:
                    pubServo.publish(actualPosition_y)
                    rate.sleep()

                ret,frame = cap.read()
                dY = get_distance(ret, frame)
                diffDeg_y = int(dY/oneDeg)
                arucoPosition_y = initialPosition_y + diffDeg_y

if __name__ == '__main__':
    try:
        distanceGenerator()
    except rospy.ROSInterruptException:
        pass
