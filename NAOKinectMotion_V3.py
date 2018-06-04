# -*- coding: utf-8 -*-
"""
Created on Wed Mar 28 20:47:49 2018

@author: Skyler
"""
import urllib2 

import math

from naoqi import ALProxy
from naoqi import motion

from pykinect import nui

import os
import win32ui


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
    
    
    RSP = ((math.atan((coordinates[9].y - coordinates[8].y)/(coordinates[9].z - coordinates[8].z))))
    LSP = ((math.atan((coordinates[5].y - coordinates[4].y)/(coordinates[5].z - coordinates[4].z))))
    
    RSR =  (math.atan(-(coordinates[9].x - coordinates[8].x)/(coordinates[9].y - coordinates[8].y)))
    LSR =  (math.atan((coordinates[5].x - coordinates[4].x)/(coordinates[5].y - coordinates[4].y)))
    
    if RSP > -0.523:
         RER = ((math.atan((coordinates[10].x - coordinates[9].x)/(coordinates[10].z - coordinates[9].z)))) 
    elif RSP < 0 and RSP > 0.523:       
        RER = ((math.atan((coordinates[10].x - coordinates[9].x)/(coordinates[10].y - coordinates[9].y))))
        RER = -(RER + math.pi/2)
    else:
        RER = 0 

    if LSP < 1.074:
        LER = -((math.atan((coordinates[6].x - coordinates[5].x)/(coordinates[6].z - coordinates[5].z))))
        LER = -(LER - math.pi/2)
    elif LSP > 0 and LSP > 0.523:
        LER = -((math.atan((coordinates[6].x - coordinates[5].x)/(coordinates[6].y - coordinates[5].y))))

        LER = -(LER - math.pi/2)


    else:
        LER = 0
        
    
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
    
    if RER < 0:
        RER = 0
    elif RER > 1.57:
        RER = 1.57
    else:
        RER = RER
        
#    if LER > 0:
#        LER = 0
#    elif LER < -1.57:
#        LER = -1.57
#    else:
#        LER = LER
    
    if LSP < -1.57:
        LSP = -1.57
    elif LSP > 1.57:
        LSP = 1.57
    else:
        LSP = LSP
        
    if RSP < -1.57:
        RSP = -1.57
    elif RSP > 1.57:
        RSP = 1.57
    else:
        RSP = RSP
    
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
       
    # Unused since the robot uses the absolute joint position now, but also
    # allows for a bit more time to position the test subject.
    
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

        
        RSP = rotation[0] #currentRArm[0] + (initialRotation[0] - rotation[0])
        RSR = -rotation[2] #currentRArm[1] + (initialRotation[2] - rotation[2])
        RER =  rotation[4]
        
        LSP = rotation[1]#currentLArm[0] + (initialRotation[1] - rotation[1])
        LSR = rotation[3] #currentRArm[1] + (initialRotation[3] - rotation[3])
        LER = -(rotation[5] - math.pi/2)
        
        print(LER, RER)
        
        RArmDeltas = [RSP, RSR, RER]
        LArmDeltas = [LSP, LSR, LER]
        

        # Moves the arms into position
        
        motionProxy.setAngles(RArmNames, RArmDeltas, fractionMaxSpeed)
        motionProxy.setAngles(LArmNames, LArmDeltas, fractionMaxSpeed)
        
        names = ["HeadYaw", "HeadPitch"]
        headCoords = headMove()
        motionProxy.setAngles(names, [-headCoords[2], headCoords[1]], fractionMaxSpeed)
        
        
        
        # Moves the arms and torso

        initialCoordinates = coordinates   


if __name__ == "__main__":
    
    try:
        win32ui.FindWindow("MixedRealityRobotics", "MixedRealityRobotics")
    except:
        print("Launching VR application...")
        workingDir = str(os.path.dirname(os.getcwd()))
        os.system("start MRR.exe " + workingDir )
        robotIP = raw_input("Input robot IP: ")
        robotIP = "127.0.0.1"
        main(robotIP)
    else:
        robotIP = raw_input("Input robot IP: ")
        robotIP = "127.0.0.1"
        main(robotIP)

    