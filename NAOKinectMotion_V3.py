# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 09:59:14 2018

@author: Skyler
"""

import almath
import motion
import argparse
from naoqi import ALProxy


from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

#import pykinect
#from pykinect import nui
#from pykinect.nui import JointId


class BodyGameRuntime(object):
    def __init__(self):

        # Loop until the user clicks the close button.
        self._done = False


        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)


        # here we will store skeleton data 
        self._bodies = None
                       


    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return



    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
    
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);

    def run(self):
                    
        # --- Cool! We have a body frame, so can get skeletons
        if self._kinect.has_new_body_frame(): 
            self._bodies = self._kinect.get_last_body_frame()

        # --- draw skeletons to _frame_surface
        if self._bodies is not None: 
            for i in range(0, self._kinect.max_body_count):
                body = self._bodies.bodies[i]
                if not body.is_tracked: 
                    continue
                joints = body.joints 
                #joint_points = [1,2,3]
                joint_points = self._kinect.body_joints_to_color_space(joints)
                
            return(joint_points)

                


                    
def main(robotIP, PORT=9559):
    motionProxy  = ALProxy("ALMotion", robotIP, PORT)
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    
    
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
    targetHead = 0
    headPos = 0
    
    motionProxy.setCollisionProtectionEnabled(effectorList[0], False)
    motionProxy.setCollisionProtectionEnabled(effectorList[1], False)

    
    #Scaling factor for position
    scaling = 3.0
    
    
    #raw_input("Press enter to capture initial pose")
    print("Please hold pose for 5 seconds")
    time.sleep(1)
    #bodyMotion.run()
    while True:
        x = bodyMotion.run()[PyKinectV2.JointType_Head].x
        y = bodyMotion.run()[PyKinectV2.JointType_Head].y
        #z = y*512 + x
        #print(str(bodyMotion.run()[PyKinectV2.JointType_Head].))
        print("X-Coord"+str(x))
        print("Y-Coord"+str(y))
#        initialCoordinates = PyKinectV2.JointType_Head.x
       # print(str(initialCoordinates))
        coordinates = []
        #bodyMotion._kinect.close()
            
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
    if coordinates[4].z - coordinates[8].z > 0.1:
        dRz_head = 0.05            
    elif coordinates[4].z - coordinates[8].z < -0.1:
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
    
    #elapsed = time.time() - t
    initialCoordinates = coordinates
    pathList = [] 
    
    """
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=50213,
                        help="Robot port number")
    
    bodyMotion = BodyGameRuntime();

    args = parser.parse_args()
    main(args.ip, args.port)
    

    