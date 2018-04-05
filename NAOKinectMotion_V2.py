# -*- coding: utf-8 -*-
"""
Created on Wed Mar 28 20:47:49 2018

@author: Skyler
"""

import almath
import motion
import argparse
from naoqi import ALProxy


import pykinect
from pykinect import nui
from pykinect.nui import JointId
import time



def main(robotIP, PORT=9559):
    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    
    
    _kinect = nui.Runtime()
    _kinect.skeleton_engine.enabled = True
    _kinect.camera.elevation_angle = 15

    # Wake up robot
    # motionProxy.wakeUp()

    # Send robot to Pose Init
    postureProxy.goToPosture("StandZero", 0.5)

    # Example showing how to use transformInterpolations
    frame        = motion.FRAME_ROBOT
    isAbsolute   = False
    useSensorValues = False
    isEnabled = True
    
    motionProxy.wbEnableEffectorControl("Head", isEnabled)

    # Motion of Arms with block process
    effectorList = ["LArm", "RArm", "Torso"]
    axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL, motion.AXIS_MASK_ALL]
    elapsed = 0.5
    targetHead = 0
    headPos = 0
    
    #Scaling factor for position
    scaling = 2.5
    
    
    
    #raw_input("Press enter to capture initial pose")
    print("Please hold pose for 5 seconds")
    #time.sleep(5)
    
    updated = False
    while updated == False:
        for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
            if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:
                initialCoordinates = skeleton.SkeletonPositions
                updated = True
                print("Initial pose saved")
                
                    
    postureProxy.goToPosture("StandZero", 0.5)
    
    # Start main update loop
    while True:
        updated = False
        
        while updated == False:
            for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
                if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:
                    coordinates = skeleton.SkeletonPositions
                    t = time.time()
                    updated = True
                elif skeleton.eTrackingState != nui.SkeletonTrackingState.TRACKED:
                    coordinates = initialCoordinates
                    updated = True
                

                    
                    
            
        timeList = [[0.15], [0.15], [0.15]] # seconds
        
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
        
        # Torso coordinates/positions
        targetTorsoTf = almath.Transform(motionProxy.getTransform("Torso", frame, useSensorValues))
        targetTorsoTf.r1_c4 += dx_Torso
        targetTorsoTf.r2_c4 += dy_Torso
        targetTorsoTf.r3_c4 += dz_Torso
        
        pathList.append(list(targetTorsoTf.toVector()))
        
        # Head rotation (Remove when using VR Headset)
        if coordinates[4].z - coordinates[8].z > 0.15:
            dRz_head = 0.05            
        elif coordinates[4].z - coordinates[8].z < -0.15:
            dRz_head = -0.05
        else:
            dRz_head = 0
        
#        if targetHead <= 2.3 and targetHead >= -2.3:
#            targetHead += dRz_head
#        else:
#            targetHead = targetHead
        
        
        headCoords = [0,0,targetHead]
        
        # Moves the arms and torso
        motionProxy.transformInterpolations(effectorList, frame, pathList,
                                     axisMaskList, timeList)
        
        # Rotates the head
        motionProxy.wbSetEffectorControl("Head", headCoords)
        
        elapsed = time.time() - t
        initialCoordinates = coordinates
        pathList = []
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=49767,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)