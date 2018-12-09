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

    horizontal_img = cv2.flip( gray, 0 )
    cv2.imshow('frame_',horizontal_img)
    cv2.waitKey(1)

    return [dX,dY]

def distanceGenerator():

    # Calculate number of pixels corresponding to 1 degree
    cap = cv2.VideoCapture(0)
    ret,frame = cap.read()
    height, width = frame.shape [:2]
    oneDeg = width/62


    # Initialize ROS environment
    pubServo_x = rospy.Publisher('servo_x', UInt16, queue_size=30)
    pubServo_y = rospy.Publisher('servo_y', UInt16, queue_size=30)
    rospy.init_node('distanceGenerator', anonymous=True)
    rate = rospy.Rate(10) # wait untill it turns (in hz)

    initialPosition_x = 40
    initialPosition_y = 90
    actualPosition_x = initialPosition_x
    actualPosition_y = initialPosition_y
    goneToInitial = False

    while not rospy.is_shutdown():

        # Go to initial position
        connections_x = pubServo_x.get_num_connections()
        connections_y = pubServo_y.get_num_connections()
        rospy.loginfo('Connections X: %d', connections_x)
        rospy.loginfo('Connections Y: %d', connections_y)

        if (connections_x > 0) and (connections_y > 0) and (goneToInitial == False):
            rospy.loginfo("Go to initial position")
            goneToInitial = True
            pubServo_x.publish(initialPosition_x)
            rate.sleep()
            pubServo_y.publish(initialPosition_y)

        # Go to aruco position
        elif (connections_x > 0) and (connections_y > 0) and (goneToInitial == True):
            rospy.loginfo("Go to aruco position")

            # Get position of aruco marker
            ret,frame = cap.read()
            dX, dY = get_distance(ret, frame)
            diffDeg_x = int(dX/oneDeg)
            diffDeg_y = int(dY/oneDeg)
            arucoPosition_x = initialPosition_x + diffDeg_x
            arucoPosition_y = initialPosition_y + diffDeg_y

            # Control loop with arucoPosition as input and actualPosition as output
            TOL = 2 # Tolerance for controller
            while ((abs(diffDeg_x) > TOL) and (abs(diffDeg_y) > TOL)):
                actualPosition_x = actualPosition_x + diffDeg_x/5
                actualPosition_y = actualPosition_y + diffDeg_y/5
                print "xxxxxxxxxxxxxxxxxxxxxxxxxxxxx: " + str(dX)
                print "yyyyyyyyyyyyyyyyyyyyyyyyyyyyy: " + str(dY)
                pubServo_x.publish(actualPosition_x)
                pubServo_y.publish(actualPosition_y)
                rate.sleep()

                ret,frame = cap.read()
                dX, dY = get_distance(ret, frame)
                diffDeg_x = int(dX/oneDeg)
                diffDeg_y = int(dY/oneDeg)
                arucoPosition_x = initialPosition_x + diffDeg_x
                arucoPosition_y = initialPosition_y + diffDeg_y

if __name__ == '__main__':
    try:
        distanceGenerator()
    except rospy.ROSInterruptException:
        pass
