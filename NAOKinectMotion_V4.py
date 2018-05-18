# -*- coding: utf-8 -*-
"""
Created on Wed May 09 09:34:57 2018

@author: Skyler
"""
# Axis Neuron Version


import almath
import socket
import argparse
import urllib2 

from naoqi import ALProxy
from naoqi import motion


import time

def axisData():
    
    TCP_IP = '127.0.0.1'
    TCP_PORT = 7003
    BUFFER_SIZE = 8192
    MESSAGE = "thing"
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    

    s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    packet = data.split('\n')
    
    for item in packet:
        strMotionData = item.split(' ')
        if len(strMotionData) == 947:
            break

            
        
    # Index 97:103 - RightHand
    # Index 235:241 - LeftHand

    
    motionData = [float(i) for i in strMotionData[0:946]]
    # Hand positions [X,Y,Z]
    handPosR = motionData[272:275]
    handPosL = motionData[648:651]
    
    # Quaternion rotations [W,X,Y,Z]
    handRotR = motionData[278:282]
    handRotL = motionData[654:658]
    
    coordinates = [handPosR, handPosL, handRotR, handRotL]
    
    return coordinates

def headMove():
    
    content = (urllib2.urlopen('http://127.0.0.1:5000').read())
    
    x,y,z = content.split(' ')
    
    headCoords = [float(z)*(3.14159/180),
                  float(x)*(3.14159/180),
                  float(y)*(3.14159/180)]
    
    for x in range(3):
        if headCoords[x] > 3.14159:
            headCoords[x] = headCoords[x] - (2*3.14159)

    print headCoords
    return headCoords
        


def main(robotIP, PORT=9559):
    
    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    

    # Wake up robot
    motionProxy.wakeUp()
    
    # Send robot to Pose Init
    
    #postureProxy.goToPosture("Crouch", 0.5)

    # Example showing how to use transformInterpolations
    frame        = motion.FRAME_TORSO
    useSensor = False
    isEnabled = True
    
    # Sets whole body balancer, collision protection, and enables head tracking
    motionProxy.wbEnable(False)
    motionProxy.setCollisionProtectionEnabled("Arms", False)
    motionProxy.wbEnableEffectorControl("Head", isEnabled)


    fractionMaxSpeed = 0.8
    
    
    # Scaling factor for position
    # NAO reach = 290mm, avg human = 750mm, scaling = human/NAO
    scaling = 2.0
    
    
    postureProxy.goToPosture("StandZero", 0.5)
    raw_input("Press enter to capture initial pose")
    print("Please hold pose for 5 seconds")
    time.sleep(2)
    
    initialCoordinates = axisData()
    
    while True:
        
        """ For the  Neuron vs the NAO, the coordinate frames are different
        
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
        
        # axisData = [handPosR, handPosL, handRotR, handRotL]
        # handPos = [X,Y,Z]
        # handRot = [W,RX,RY,RZ] (quaternion)
        coordinates = axisData()
        
        dRy_L = coordinates[3][1]-initialCoordinates[3][1]
        dRy_R = coordinates[2][1]-initialCoordinates[2][1]
        
        dRz_L = coordinates[3][2]-initialCoordinates[3][2]
        dRz_R = coordinates[2][2]-initialCoordinates[2][2]
    
        dRx_L = coordinates[3][3]-initialCoordinates[3][3]
        dRx_R = coordinates[2][3]-initialCoordinates[2][3]
        
        
        currentLArm = motionProxy.getPosition("LArm", frame, useSensor)
        currentRArm = motionProxy.getPosition("RArm", frame, useSensor)
        
        
        # Arm position Tracking
        
        dy_L = (coordinates[1][0]-initialCoordinates[1][0])/scaling
        dy_R = (coordinates[0][0]-initialCoordinates[0][0])/scaling
        
        
#        print('X-Coords R ' + str(dx_R) + '\n')
#        print('X-Coords L ' + str(dx_L))
        
        dz_L = (coordinates[1][1]-initialCoordinates[1][1])/scaling
        dz_R = (coordinates[0][1]-initialCoordinates[0][1])/scaling
        
    
        
        dx_L = (coordinates[1][2]-initialCoordinates[1][2])/scaling
        dx_R = (coordinates[0][2]-initialCoordinates[0][2])/scaling
        
        
        
        # Rotational Data
        
        
        RArmTarget = [currentRArm[0] + dx_R,
                      currentRArm[1] + dy_R,
                      currentRArm[2] + dz_R,
                      currentRArm[3] + 0.0,
                      currentRArm[4] + 0.0,
                      currentRArm[5] + 0.0]
        """
                      currentRArm[3] + dRx_R,
                      currentRArm[4] + dRy_R,
                      currentRArm[5] + dRz_R]
        """
                      
                      
        LArmTarget = [currentLArm[0] + dx_L,
                      currentLArm[1] + dy_L,
                      currentLArm[2] + dz_L,
                      currentLArm[3] + 0.0,
                      currentLArm[4] + 0.0,
                      currentLArm[5] + 0.0]
        """
                      currentLArm[3] + dRx_L,
                      currentLArm[4] + dRy_L,
                      currentLArm[5] + dRz_L]
        """
    
    
                # Rotates the head
        names = ["HeadYaw", "HeadPitch"]
        headCoords = headMove()
        motionProxy.setAngles(names, [-headCoords[2], headCoords[1]], fractionMaxSpeed)
       # headCoords = [0,0,targetHead]
        
        # Moves the arms and torso
        motionProxy.setPositions("LArm", frame, LArmTarget, fractionMaxSpeed, 63)
        motionProxy.setPositions("RArm", frame, RArmTarget, fractionMaxSpeed, 63)
        

        
        initialCoordinates = coordinates
        
        time.sleep(0.2)

    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
