# -*- coding: utf-8 -*-
"""
Created on Wed Apr 25 15:03:32 2018

@author: Skyler
"""

import urllib2 

import argparse
import motion
import almath
from naoqi import ALProxy
from time import sleep
def main():
    
    robotIP = '169.254.129.162'
    PORT = 9559

    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    
    
    # Example showing how to use transformInterpolations
    frame        = motion.FRAME_ROBOT
    isAbsolute   = True
    useSensorValues = True
    isEnabled = True
    
    # Wake up robot
    motionProxy.wakeUp()
    motionProxy.setCollisionProtectionEnabled("Arms", False)
    

    
    postureProxy.goToPosture("Crouch", 0.4)
    motionProxy.wbEnable(True)
    motionProxy.wbEnableEffectorControl("Head", isEnabled)


    while True:
        
        content = (urllib2.urlopen('http://138.67.229.47:5000').read())
        
        x,y,z = content.split(' ')
        
        headCoords = [float(z)*(3.14159/180),
                      float(x)*(3.14159/180),
                      float(y)*(3.14159/180)]
        
        for x in range(3):
            if headCoords[x] > 3.14159:
                headCoords[x] = headCoords[x] - (2*3.14159)

        
        #print headCoords
        
        
        # Rotates the head
        
        motionProxy.wbSetEffectorControl("Head", headCoords)
        sleep(0.1)

if __name__ == "__main__":
    main()