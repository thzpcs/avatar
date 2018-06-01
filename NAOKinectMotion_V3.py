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
    
    
    RSP = -((math.atan((coordinates[9].y - coordinates[8].y)/(coordinates[9].z - coordinates[8].z))))
    LSP = -((math.atan((coordinates[5].y - coordinates[4].y)/(coordinates[5].z - coordinates[4].z))))
    
    RSR =  (math.atan(-(coordinates[9].x - coordinates[8].x)/(coordinates[9].y - coordinates[8].y)))
    LSR =  (math.atan((coordinates[5].x - coordinates[4].x)/(coordinates[5].y - coordinates[4].y)))
    
    RER = ((math.atan((coordinates[10].y - coordinates[9].y)/(coordinates[10].z - coordinates[9].z))))
    LER = -((math.atan((coordinates[6].y - coordinates[5].y)/(coordinates[6].z - coordinates[5].z))))
    
#    RSP = -(math.atan2((coordinates[9].z - coordinates[8].z), (coordinates[9].y - coordinates[8].y))) - math.pi/2
#    LSP = -(math.atan2((coordinates[5].z - coordinates[4].z), (coordinates[5].y - coordinates[4].y))) - math.pi/2
#    
#    RSR =  -(math.atan2((coordinates[9].y - coordinates[8].y),(coordinates[9].x - coordinates[8].x)))
#    LSR =  (math.atan2((coordinates[5].x - coordinates[4].x),(coordinates[5].y - coordinates[4].y)))


    print(LER, RER)
    
    if LSR < -0.31:
        LSR = -0.31
    elif LSR > 1.31:
        LSR = 1.31
    else:
        LSR = LSR
        
    if RSR < 0:
        RSR = 0
    elif RSR > 1.31:
        RSR = 1.31
    else:
        RSR = RSR
    
    if RER > 0:
        RER = 0
    elif RER > 1.55:
        RER = 1.55
    else:
        RER = RER
    
#    if LSP < -1.57:
#        LSP = -1.57
#    elif LSP > 1.57:
#        LSP = 1.57
#    else:
#        LSP = LSP
#        
#    if RSP < -1.57:
#        RSP = -1.57
#    elif RSP > 1.57:
#        RSP = 1.57
#    else:
#        RSP = RSP
    
    rotation = [RSP, LSP, RSR, LSR, RER, LER]
    
    return rotation
    
        

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
 
    frame = motion.FRAME_ROBOT
    useSensor = False


    motionProxy.wbEnable(False)

    fractionMaxSpeed = 0.9
    
    RArmNames = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll"]
    LArmNames = ["LShoulderPitch", "LShoulderRoll", "LElbowRoll"]
    
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
                initialRotation = armRotation(initialCoordinates)
                updated = True
                print("Initial pose saved")

    while True:
        
        coordinates = kinectData()
        rotation = armRotation(coordinates)
        
        
        # Calculates the angle deltas
        # Joint names/orders:
        # RArmNames = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"]
        # LArmNames = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
        
        # dRSP = deltaRightShoulderPitch, dLSR = deltaLeftShoulderRoll, etc
        
        currentRArm = motionProxy.getAngles("RArm", useSensor)
        currentLArm = motionProxy.getAngles("LArm", useSensor)
        
        RSP = -rotation[0] #currentRArm[0] + (initialRotation[0] - rotation[0])
        RSR = -rotation[2] #currentRArm[1] + (initialRotation[2] - rotation[2])
        RER = -(rotation[4]+math.pi)
        
        LSP = -rotation[1]#currentLArm[0] + (initialRotation[1] - rotation[1])
        LSR = rotation[3] #currentRArm[1] + (initialRotation[3] - rotation[3])
        LER = -rotation[5]-math.pi/2
        
        
        RArmDeltas = [RSP, RSR, RER]
        LArmDeltas = [LSP, LSR, LER]
#        # Sets the delta arm positions        
#        RArmTarget = [currentRArm[0] + dx_R,
#                      currentRArm[1] + dy_R,
#                      currentRArm[2] + dz_R,
#                      currentRArm[3] + 0.00,
#                      currentRArm[4] + 0.00,
#                      currentRArm[5] + drZ_R]
#        
#        print(rotation)
#                      
#        LArmTarget = [currentLArm[0] + dx_L,
#                      currentLArm[1] + dy_L,
#                      currentLArm[2] + dz_L,
#                      currentLArm[3] + 0.00,
#                      currentLArm[4] + 0.00,
#                      currentLArm[5] + drZ_L]

        

        # Moves the arms into position
        
        motionProxy.setAngles(RArmNames, RArmDeltas, fractionMaxSpeed)
        motionProxy.setAngles(LArmNames, LArmDeltas, fractionMaxSpeed)
        
        names = ["HeadYaw", "HeadPitch"]
        headCoords = headMove()
        motionProxy.setAngles(names, [-headCoords[2], headCoords[1]], fractionMaxSpeed)
        
        
        
        # Moves the arms and torso
#        motionProxy.setPositions("LArm", frame, LArmTarget, fractionMaxSpeed, 63)
#        motionProxy.setPositions("RArm", frame, RArmTarget, fractionMaxSpeed, 63)

        initialCoordinates = coordinates
        initialRotation = rotation
        


    
if __name__ == "__main__":
#    parser = argparse.ArgumentParser()
#    parser.add_argument("--ip", type=str, default="169.254.129.162",
#                        help="Robot ip address")
#    parser.add_argument("--port", type=int, default=9559,
#                        help="Robot port number")
#
#    args = parser.parse_args()
    
    #robotIP = raw_input("Input robot IP: ")
    robotIP = "127.0.0.1"
    main(robotIP)

    