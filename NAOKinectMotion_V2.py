# -*- coding: utf-8 -*-
"""
Created on Wed Mar 28 20:47:49 2018

@author: Skyler
"""

import almath

import argparse

from naoqi import ALProxy
from naoqi import motion


import pykinect
from pykinect import nui
from pykinect.nui import JointId

import time



def main(robotIP, PORT=9559):
    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    
    
    _kinect = nui.Runtime()
    _kinect.skeleton_engine.enabled = True
    _kinect.camera.elevation_angle = 10

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Pose Init
    
    #postureProxy.goToPosture("Crouch", 0.5)

    # Example showing how to use transformInterpolations
    frame        = motion.FRAME_ROBOT
    isAbsolute   = True
    useSensorValues = True
    isEnabled = True

    motionProxy.wbEnable(isEnabled)
    motionProxy.wbEnableEffectorControl("Head", isEnabled)

    # Motion of Arms with block process
    effectorList = ["LArm", "RArm"]#, "Torso"]
    axisMaskList = [motion.AXIS_MASK_ALL, motion.AXIS_MASK_ALL]#, motion.AXIS_MASK_ALL]
    elapsed = 0.5
    targetHead = 0
    headPos = 0
    
    
    
    # Leg variables
    X_walk = 0.0
    Y_walk = 0.0
    Theta_walk = 0.0
    Frequency = 1.0
    
    # Scaling factor for position
    # NAO reach = 290mm, avg human = 750mm, scaling = human/NAO
    scaling = 2.0
    
    
    postureProxy.goToPosture("StandZero", 1.0)
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
                
                    
    #postureProxy.goToPosture("StandZero", 0.5)
    
    # Start main update loop
    while True:
        
        updated = False
        
        while updated == False:
            for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
                if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:
                    coordinates = skeleton.SkeletonPositions
                    updated = True
                    
#                elif skeleton.eTrackingState != nui.SkeletonTrackingState.TRACKED:
#                    coordinates = initialCoordinates
#                    print("Not tracking")
#                    updated = True
                    
            
        timeList = [[0.5], [0.5]]#, [0.1]] # seconds
        
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
        
        # Arm position Tracking
        dx_L = -(coordinates[7].z-initialCoordinates[7].z)/scaling
        dx_R = -(coordinates[11].z-initialCoordinates[11].z)/scaling
        
#        print('X-Coords R ' + str(dx_R))
#        print('X-Coords L ' + str(dx_L))
        
        dy_L = -(coordinates[7].x-initialCoordinates[7].x)/scaling
        dy_R = -(coordinates[11].x-initialCoordinates[11].x)/scaling
        
#        print('Y-Coords R ' + str(dy_R))
#        print('Y-Coords L ' + str(dy_L))
        
        dz_L = (coordinates[7].y-initialCoordinates[7].y)/scaling
        dz_R = (coordinates[11].y-initialCoordinates[11].y)/scaling
        
#        print('Z-Coords R ' + str(dz_R))
#        print('Z-Coords L ' + str(dz_L))
        
        dx_Torso = -(coordinates[2].z-initialCoordinates[2].z)/scaling
        dy_Torso = -(coordinates[2].x-initialCoordinates[2].x)/scaling
        dz_Torso = (coordinates[2].y-initialCoordinates[2].y)/scaling
        
#        # Leg Position Tracking
#        dy_LegR = (coordinates[13].y-initialCoordinates[13].y)
#        dy_LegL = (coordinates[17].y-initialCoordinates[17].y)
#        
#        # If leg is raised, start walking
#        if dy_LegR > 0.5 or dy_LegL > 0.5:
#            y_legR = dy_LegR
#            y_legL = dy_LegR
#            
#            time.sleep(1.0)
#            print("Entering walk mode")
#            motionProxy.moveInit()
#            
#            print("Hold single leg to rotate (L = CW, R = CCW) or both for forward motion")
#            time.sleep(1.0)
            
#            for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
#                if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:
#                    coordinates = skeleton.SkeletonPositions
#            
#            
#            if (y_legR-coordinates[13].y) < 0 and (y_legL-coordinates[17].y) > 0.5:
#                print("Moving forward")
#                X_walk = 1.0
#            elif (y_legR-coordinates[13].y) < 0.01 and (y_legR-coordinates[13].y) > 0.01:
#                print("Rotating CCW")
#                Theta_walk = -0.523
#            elif (y_legL-coordinates[17].y) < 0.01 and (y_legL-coordinates[17].y) > 0.01:
#                print("Rotating CW")
#                Theta_walk = 0.523
#            
#            time.sleep(1.0)
#            motionProxy.setWalkTargetVelocity(X_walk, Y_walk, Theta_walk, Frequency, [["MaxStepX", 0.06]])
#            time.sleep(2.0)
#            print("walk Speed X :",motionProxy.getRobotVelocity()[0]," m/s")
#            motionProxy.stopMove()
            
#        else:
#            continue
        
        pathList = []
        
        # Left arm coordinates/positions
        targetLArmTf = almath.Transform(motionProxy.getTransform("LArm", frame, useSensorValues))
        targetLArmTf.r1_c4 += dx_L
        targetLArmTf.r2_c4 += dy_L
        targetLArmTf.r3_c4 += dz_L
        
        
        pathList.append(list(targetLArmTf.toVector()))
        
        # Right arm coordinates/positions
        targetRArmTf = almath.Transform(motionProxy.getTransform("RArm", frame, useSensorValues))
        targetRArmTf.r1_c4 += dx_R
        targetRArmTf.r2_c4 += dy_R
        targetRArmTf.r3_c4 += dz_R
        
        pathList.append(list(targetRArmTf.toVector()))
        
        """
        # Torso coordinates/positions
        targetTorsoTf = almath.Transform(motionProxy.getTransform("Torso", frame, useSensorValues))
        targetTorsoTf.r1_c4 += dx_Torso
        targetTorsoTf.r2_c4 += dy_Torso
        targetTorsoTf.r3_c4 += dz_Torso
        
        pathList.append(list(targetTorsoTf.toVector()))
        """

        
        
        headCoords = [0,0,targetHead]
        
        # Moves the arms and torso
        motionProxy.transformInterpolations(effectorList, frame, pathList,
                                     axisMaskList, timeList, isAbsolute)
        
        # Rotates the head
        #motionProxy.wbSetEffectorControl("Head", headCoords)
        
        #elapsed = time.time() - t
        initialCoordinates = coordinates
        pathList = []
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="169.254.129.162",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)