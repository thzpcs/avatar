# -*- coding: utf-8 -*-
"""
Created on Fri Mar 02 15:48:43 2018

@author: Skyler
"""

import argparse
import motion
import almath
from naoqi import ALProxy

# Import of pykinect modules "
from visual import *
import pykinect
from pykinect import nui
from pykinect.nui import JointId


# Initialize and level the Kinect sensor.
_kinect = nui.Runtime()
_kinect.skeleton_engine.enabled = True
_kinect.camera.elevation_angle = 20

class Skeleton:
    """Kinect skeleton represented as a VPython frame.
    """

    def __init__(self, f):
        """Create a skeleton in the given VPython frame f.
        """
        self.frame = f
        

    def update(self):
        """Update the skeleton joint positions in the depth sensor frame.

        Return true iff the most recent sensor frame contained a tracked
        skeleton.
        """
        updated = False
        for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
            if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:

                # Move the joints.
                for joint, p in zip(self.joints, skeleton.SkeletonPositions):
                    joint.pos = (p.x, p.y, p.z)

                # Move the bones.
                for bone, bone_id in zip(self.bones, _bone_ids):
                    p1, p2 = [self.joints[id].pos for id in bone_id]
                    bone.pos = p1
                    bone.axis = p2 - p1
                updated = True
        return updated
    
"""
def rightArm():
    
def leftArm():
    
def torso():
    
def head():
    """
    

def main(robotIP, PORT=9559):

    # A bone is a cylinder connecting two joints, each specified by an id.
    # Probably won't need this, inclused from vpykinect just to make sure things will work
    _bone_ids = [[0, 1], [1, 2], [2, 3], [7, 6], [6, 5], [5, 4], [4, 2],
                 [2, 8], [8, 9], [9, 10], [10, 11], [15, 14], [14, 13], 
                 [13, 12], [12, 0], [0, 16], [16, 17], [17, 18], [18, 19]]
    


    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

    # Wake up robot
    motionProxy.wakeUp()
    

    
    # Send robot to Stand Init
    postureProxy.goToPosture("StandInit", 0.5)

    


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)