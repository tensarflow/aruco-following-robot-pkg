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

    gray = aruco.drawDetectedMarkers(gray, corners)

    cv2.imshow('frame',gray)
    cv2.waitKey(1)
    #return [dX,dY]
    return dX

def distanceGenerator():
    cap = cv2.VideoCapture(0)
    ret,frame = cap.read()
    pubServo_1Servo_1 = rospy.pubServo_1Servo_1lisher('servo', UInt16, queue_size=10)
    rospy.init_node('distanceGenerator', anonymous=True)
    rate = rospy.Rate(0.8) # in hz

    # Number of pixels corresponding to 1 degree
    height, width = frame.shape [:2]
    oneDeg = width/62

    initialPosition = 90 # initial position of servo

    while not rospy.is_shutdown():
        # Get the image
        ret,frame = cap.read()

        # Get the distance from the image
        distanceX = get_distance(ret, frame)
        toTurn = initialPosition + int(distanceX/oneDeg)

        if toTurn < 10:
            return
        # pubServo_1Servo_1lish to ROS environment
        rospy.loginfo(toTurn)
        pubServo_1Servo_1.pubServo_1Servo_1lish(toTurn)
        rate.sleep()

if __name__ == '__main__':
    try:
        distanceGenerator()
    except rospy.ROSInterruptException:
        pass
