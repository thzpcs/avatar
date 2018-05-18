# -*- coding: utf-8 -*-
"""
Created on Sat May 05 12:53:04 2018

@author: Skyler
"""

# -*- coding: utf-8 -*-
"""
Created on Wed Mar 28 20:47:49 2018

@author: Skyler
"""


import argparse

from naoqi import ALProxy
from naoqi import motion

import socket
import time



def main(robotIP, PORT=9559):
    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    
    
    TCP_IP = '127.0.0.1'
    TCP_PORT = 7001
    BUFFER_SIZE = 2048

    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))

    # Wake up robot
    motionProxy.wakeUp()

    # Send robot to Pose Init
    
    #postureProxy.goToPosture("Crouch", 0.5)

    # Example showing how to use transformInterpolations
    frame        = motion.FRAME_ROBOT
    useSensor = True
    isEnabled = True

    motionProxy.wbEnable(False)
    motionProxy.wbEnableEffectorControl("Head", isEnabled)


    fractionMaxSpeed = 0.5
    
    
    # Scaling factor for position
    # NAO reach = 290mm, avg human = 750mm, scaling = human/NAO
    scaling = 0.3
    
    
    postureProxy.goToPosture("StandZero", 0.5)
    raw_input("Press enter to capture initial pose")
    print("Please hold pose for 5 seconds")
    time.sleep(5)
                    
    #postureProxy.goToPosture("StandZero", 0.5)
    
    # Start main update loop
    while True:
        
        data = s.recv(BUFFER_SIZE)
        rawCoords = data.split(' ')
        
        currentLArm = motionProxy.getPosition("LArm", frame, useSensor)
        currentRArm = motionProxy.getPosition("RArm", frame, useSensor)
        
        # Arm position Tracking
        dx_L = -(coordinates[7].z-initialCoordinates[7].z)/scaling
        dx_R = -(coordinates[11].z-initialCoordinates[11].z)/scaling
        

        
        print('X-Coords R ' + str(dx_R) + '\n')
        print('X-Coords L ' + str(dx_L))
        
        dy_L = -(coordinates[7].x-initialCoordinates[7].x)/scaling
        dy_R = -(coordinates[11].x-initialCoordinates[11].x)/scaling
        

        
        dz_L = (coordinates[7].y-initialCoordinates[7].y)/scaling
        dz_R = (coordinates[11].y-initialCoordinates[11].y)/scaling
        

        
        dx_Torso = -(coordinates[2].z-initialCoordinates[2].z)/scaling
        dy_Torso = -(coordinates[2].x-initialCoordinates[2].x)/scaling
        dz_Torso = (coordinates[2].y-initialCoordinates[2].y)/scaling
        
        
        RArmTarget = [currentRArm[0] + dx_R,
                      currentRArm[1] + dy_R,
                      currentRArm[2] + dz_R,
                      currentRArm[3] + 0.0,
                      currentRArm[4] + 0.0,
                      currentRArm[5] + 0.0,]
        
                      
                      
        LArmTarget = [currentLArm[0] + dx_L,
                      currentLArm[1] + dy_L,
                      currentLArm[2] + dz_L,
                      currentLArm[3] + 0.0,
                      currentLArm[4] + 0.0,
                      currentLArm[5] + 0.0,]

        
        """
        # Torso coordinates/positions
        targetTorsoTf = almath.Transform(motionProxy.getTransform("Torso", frame, useSensorValues))
        targetTorsoTf.r1_c4 += dx_Torso
        targetTorsoTf.r2_c4 += dy_Torso
        targetTorsoTf.r3_c4 += dz_Torso
        
        pathList.append(list(targetTorsoTf.toVector()))
        """


        
       # headCoords = [0,0,targetHead]
        
        # Moves the arms and torso
        motionProxy.setPositions("LArm", frame, LArmTarget, fractionMaxSpeed, 63)
        #motionProxy.setPositions("RArm", frame, RArmTarget, fractionMaxSpeed, 63)
        
        # Rotates the head
        #motionProxy.wbSetEffectorControl("Head", headCoords)
        
        #elapsed = time.time() - t
        initialCoordinates = coordinates
        
        time.sleep(0.1)

    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)