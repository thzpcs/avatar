# -*- coding: utf-8 -*-
"""
Created on Mon Mar 19 11:04:48 2018

@author: Skyler
"""

from visual import *
from pykinect import nui
from pykinect.nui import JointId
import time
# Import of NAOqi Modules
import argparse
import motion
import almath
from naoqi import ALProxy



class Skeleton:
    """Kinect skeleton represented as a VPython frame.
    """

    def __init__(self, f):
        """Create a skeleton in the given VPython frame f.
        """
        self.frame = f
#        self.joints = [sphere(frame=f, radius=0.08, color=color.yellow)
#                       for i in range(20)]
#        self.joints[3].radius = 0.125
#        self.bones = [cylinder(frame=f, radius=0.05, color=color.yellow)
#                      for bone in _bone_ids]
        self.jointCoords = []
        
    def update(self):
        """Update the skeleton joint positions in the depth sensor frame.

        Return true iff the most recent sensor frame contained a tracked
        skeleton.
        """
        updated = False
        for skeleton in _kinect.skeleton_engine.get_next_frame().SkeletonData:
            if skeleton.eTrackingState == nui.SkeletonTrackingState.TRACKED:

#                # Move the joints.
#                for joint, p in zip(self.joints, skeleton.SkeletonPositions):
#                    joint.pos = (p.x, p.y, p.z)
#                    
                self.jointCoords = skeleton.SkeletonPositions
                    
                # Move the bones.
#                for bone, bone_id in zip(self.bones, _bone_ids):
#                    p1, p2 = [self.joints[id].pos for id in bone_id]
#                    bone.pos = p1
#                    bone.axis = p2 - p1
                updated = True
        return updated
    
    
def draw_sensor(f):
    """Draw 3D model of the Kinect sensor.

    Draw the sensor in the given (and returned) VPython frame f, with
    the depth sensor frame aligned with f.
    """
    box(frame=f, pos=(0, 0, 0), length=0.2794, height=0.0381, width=0.0635,
        color=color.blue)
    cylinder(frame=f, pos=(0, -0.05715, 0), axis=(0, 0.0127, 0), radius=0.0381,
             color=color.blue)
    cone(frame=f, pos=(0, -0.04445, 0), axis=(0, 0.01905, 0), radius=0.0381,
         color=color.blue)
    cylinder(frame=f, pos=(0, -0.05715, 0), axis=(0, 0.0381, 0), radius=0.0127,
             color=color.blue)
    cylinder(frame=f, pos=(-0.0635, 0, 0.03175), axis=(0, 0, 0.003),
             radius=0.00635, color=color.red)
    cylinder(frame=f, pos=(-0.0127, 0, 0.03175), axis=(0, 0, 0.003),
             radius=0.00635, color=color.red)
    cylinder(frame=f, pos=(0.0127, 0, 0.03175), axis=(0, 0, 0.003),
             radius=0.00635, color=color.red)
    text(frame=f, text='KINECT', pos=(0.06985, -0.00635, 0.03175),
         align='center', height=0.0127, depth=0.003)
    return f

# A bone is a cylinder connecting two joints, each specified by an id.
_bone_ids = [[0, 1], [1, 2], [2, 3], [7, 6], [6, 5], [5, 4], [4, 2],
             [2, 8], [8, 9], [9, 10], [10, 11], [15, 14], [14, 13], [13, 12],
             [12, 0], [0, 16], [16, 17], [17, 18], [18, 19]]



# Initialize and level the Kinect sensor.
print('Initialize Kinect \n')
_kinect = nui.Runtime()
_kinect.skeleton_engine.enabled = True
_kinect.camera.elevation_angle = 15
        
### main robot loop 
class NAO:
    def __init__(self):
        
        ''' Example showing a path of two positions
        Warning: Needs a PoseInit before executing
        '''
        print('Initializing NAO \n')
        robotIP = '127.0.0.1'
        PORT=9559
        
        self.motionProxy  = ALProxy("ALMotion", robotIP, PORT)
        self.postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    
        # Wake up robot
        self.motionProxy.wakeUp()
    
        # Send robot to Stand Init
        self.postureProxy.goToPosture("StandInit", 0.5)
        
        
    def move(self,coordinates):
        #Kinect [11] = RightHand, [7] = LeftHand
        
        motionProxy = self.motionProxy
        # Example showing how to use transformInterpolations
        frame        = motion.FRAME_ROBOT
        isAbsolute   = False
        useSensorValues = False
    
        # Motion of Arms with block process
        effectorList = ["LHand", "RHand"]
        axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL]
        timeList     = [[1.0], [1.0]]         # seconds
        
        print('Moving hands')
        dx_R = coordinates[7].x
        dx_L = coordinates[11].x
        
#        dy_R = coords[7].y
#        dy_L = coords[11].y
#        
#        dz_R = coords[7].z
#        dz_L = coords[11].z


        pathList = []
        targetLArmTf = almath.Transform(motionProxy.getTransform("LHand", frame, useSensorValues))
        
        targetLArmTf.r1_c4 += dx_L
#        targetLArmTf.r2_c4 += dy_L
#        targetLArmTf.r3_c4 += dz_L
        
        pathList.append(list(targetLArmTf.toVector()))
    
        targetRArmTf = almath.Transform(motionProxy.getTransform("RHand", frame, useSensorValues))
        
        targetRArmTf.r1_c4 += dx_R
#        targetRArmTf.r2_c4 += dy_R
#        targetRArmTf.r3_c4 += dz_R
        
        pathList.append(list(targetRArmTf.toVector()))
    
        motionProxy.transformInterpolations(effectorList, frame, pathList,
                                     axisMaskList, timeList)
    
#        # Motion of Arms and Torso with block process
#        effectorList = ["LArm", "RArm", "Torso"]
#        axisMaskList = [motion.AXIS_MASK_VEL,
#                        motion.AXIS_MASK_VEL,
#                        motion.AXIS_MASK_ALL]
#        timeList     = [[4.0],
#                        [4.0],
#                        [1.0, 2.0, 3.0, 4.0]] # seconds
#    
#        dx = 0.03 # translation axis X (meters)
#        dy = 0.05 # translation axis Y (meters)
#    
#        pathList = []
#    
#        targetLArmTf = almath.Transform(motionProxy.getTransform("LArm", frame, useSensorValues))
#        pathList.append(list(targetLArmTf.toVector()))
#    
#        targetRArmTf = almath.Transform(motionProxy.getTransform("RArm", frame, useSensorValues))
#        pathList.append(list(targetRArmTf.toVector()))
    
#        pathTorsoList = []
#        # point 1
#        initTorsoTf = almath.Transform(motionProxy.getTransform("Torso", frame, useSensorValues))
#        targetTorsoTf = initTorsoTf
#        targetTorsoTf.r2_c4 += dy
#        pathTorsoList.append(list(targetTorsoTf.toVector()))
#    
#        # point 2
#        initTorsoTf = almath.Transform(motionProxy.getTransform("Torso", frame, useSensorValues))
#        targetTorsoTf = initTorsoTf
#        targetTorsoTf.r1_c4 += -dx
#        pathTorsoList.append(list(targetTorsoTf.toVector()))
#    
#        # point 3
#        initTorsoTf = almath.Transform(motionProxy.getTransform("Torso", frame, useSensorValues))
#        targetTorsoTf = initTorsoTf
#        targetTorsoTf.r2_c4 += -dy
#        pathTorsoList.append(list(targetTorsoTf.toVector()))
#    
#        # point 4
#        initTorsoTf = almath.Transform(motionProxy.getTransform("Torso", frame, useSensorValues))
#        pathTorsoList.append(list(initTorsoTf.toVector()))
#    
#        pathList.append(pathTorsoList)
#    
#        motionProxy.transformInterpolations(effectorList, frame, pathList,
#                                      axisMaskList, timeList)

        
        #print(str(coords[2]))
    
        # Go to rest position
        #motionProxy.rest()


#def main():
#    return();




if __name__ == '__main__':

    
#    parser = argparse.ArgumentParser()
#    parser.add_argument("--ip", type=str, default="127.0.0.1",
#                        help="Robot ip address")
#    parser.add_argument("--port", type=int, default=9559,
#                        help="Robot port number")
#    
#    args = parser.parse_args()
#    
    print('Initializing VPy \n')
    draw_sensor(frame())
    skeleton = Skeleton(frame(visible=False))

    moveJoints = NAO()
    NAO.__init__
    
    while True:
        rate(30)
        skeleton.frame.visible = skeleton.update()
        
        while coords == []:
            time.sleep(1)
            #print('Waiting for coordinates')
            
        #print('Initializing movement \n')

    
    