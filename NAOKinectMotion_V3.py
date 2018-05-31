# -*- coding: utf-8 -*-
"""
Created on Wed Mar 28 20:47:49 2018

@author: Skyler
"""
import urllib2 

import socket
import argparse

import math
from naoqi import ALProxy
from naoqi import motion

from pykinect import nui

import time

def headMove():
    
    content = (urllib2.urlopen('http://127.0.0.1:5000').read())
    
    x,y,z = content.split(' ')
    
    headCoords = [float(z)*(3.14159/180),
                  float(x)*(3.14159/180),
                  float(y)*(3.14159/180)]
    
    for x in range(3):
        if headCoords[x] > 3.14159:
            headCoords[x] = headCoords[x] - (2*3.14159)

    return headCoords


def kinectData():
    """ For the Kinect vs the NAO, the coordinate frames are different
    
    On the NAO, in front of the robot is the +X dir, and behind is -X dir.
    On the Kinect, +X is to the right of the Kinect, and -X  is to the left
    
    On the NAO, +Y is to the left of the robot, and -Y is to the right
    On the Kinect, +Y is above the Kinect, and -Y is below the Kinect
    
    
    On the NAO, +Z is above the robot, -Z is below
    On the Kinect, +Z is away from the Kinect, and -Z is towards the Kinect
    
    In short, to take Kinect coordinates into the NAO's coordinates:
        K_X == N_Y
        K_Y == N_Z
        K_Z == N_X
    """           
     
    # Updates the current skeleton data and joint coordinates
    updated = False
    while updated == False:
        for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
            if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:
                coordinates = skeleton.SkeletonPositions
                updated = True
            else:
                updated == False
                
    return coordinates

def armRotation(coordinates):
    
    
        

def main(robotIP, PORT=9559):
    
    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
        
    
    global _kinect
    
    # Enables the Kinect
    
    _kinect = nui.Runtime()
    _kinect.skeleton_engine.enabled = True
    _kinect.camera.elevation_angle = 5

    # Wake up robot
    motionProxy.wakeUp()
 
    frame        = motion.FRAME_ROBOT
    useSensor = False
    isEnabled = True

    motionProxy.wbEnable(False)

    fractionMaxSpeed = 0.9
    
    
    # Scaling factor for position
    # NAO reach = 290mm, avg human = 750mm, scaling = human/NAO
    scaling = 2.0
    
    
    postureProxy.goToPosture("StandZero", 0.5)
    raw_input("Press enter to capture initial pose")
    print("Please hold pose for 5 seconds")
    time.sleep(5)
    
    updated = False
    while updated == False:
        for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
            if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:
                initialCoordinates = skeleton.SkeletonPositions
                updated = True
                print("Initial pose saved")

    while True:
        
        coordinates = kinectData()
        rotation = armRotation(coordinates)
        
        
        currentLArm = motionProxy.getPosition("LArm", frame, useSensor)
        currentRArm = motionProxy.getPosition("RArm", frame, useSensor)
        
        # Arm position Tracking
        dx_L = -(coordinates[7].z-initialCoordinates[7].z)/scaling
        dx_R = -(coordinates[11].z-initialCoordinates[11].z)/scaling
        
#        print('X-Coords R ' + str(dx_R) + '\n')
#        print('X-Coords L ' + str(dx_L))
        
        dy_L = -(coordinates[7].x-initialCoordinates[7].x)/scaling
        dy_R = -(coordinates[11].x-initialCoordinates[11].x)/scaling
        

        
        dz_L = (coordinates[7].y-initialCoordinates[7].y)/scaling
        dz_R = (coordinates[11].y-initialCoordinates[11].y)/scaling
        

        

        # Sets the delta arm positions        
        RArmTarget = [currentRArm[0] + dx_R,
                      currentRArm[1] + dy_R,
                      currentRArm[2] + dz_R,
                      currentRArm[3] + 0.00,
                      currentRArm[4] + 0.00,
                      currentRArm[5] + 0.00]
        
                      
                      
        LArmTarget = [currentLArm[0] + dx_L,
                      currentLArm[1] + dy_L,
                      currentLArm[2] + dz_L,
                      currentLArm[3] + 0.00,
                      currentLArm[4] + 0.00,
                      currentLArm[5] + 0.00]

        

        
        names = ["HeadYaw", "HeadPitch"]
        headCoords = headMove()
        motionProxy.setAngles(names, [-headCoords[2], headCoords[1]], fractionMaxSpeed)
        
        # Moves the arms and torso
        motionProxy.setPositions("LArm", frame, LArmTarget, fractionMaxSpeed, 63)
        motionProxy.setPositions("RArm", frame, RArmTarget, fractionMaxSpeed, 63)

        initialCoordinates = coordinates
        
        #time.sleep(0.1)

    
if __name__ == "__main__":
#    parser = argparse.ArgumentParser()
#    parser.add_argument("--ip", type=str, default="169.254.129.162",
#                        help="Robot ip address")
#    parser.add_argument("--port", type=int, default=9559,
#                        help="Robot port number")
#
#    args = parser.parse_args()
    
    robotIP = raw_input("Input robot IP: ")
    main(robotIP)

    