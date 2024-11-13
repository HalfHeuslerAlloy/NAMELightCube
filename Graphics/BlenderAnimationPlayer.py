# -*- coding: utf-8 -*-
"""
Created on Wed Nov 13 15:47:32 2024

@author: eenmv
"""

import numpy as np
import lightCubeUtil
import serial

CommPortID = 6

LightCube = np.zeros([16,16,16,3],dtype='bool')
LightCubeOld = np.copy(LightCube)

Frames = LightCube.loadBlenderAnimation(r"C:\Users\eenmv\Documents\Github\NAMELightCube\Blender Animation Scene\temp1_LargeCube.txt")


cubePort = serial.Serial("COM"+str(CommPortID),115200)

for F in range(100):
    
    LightCube = np.zeros([16,16,16,3],dtype = 'bool')
    LightCube = LightCube.blenderDraw(LightCube,Frames,F)

    packet = LightCube.getUpdatedVoxels(LightCube, LightCubeOld)
    
    LightCubeOld = np.copy(LightCube)
    
    if cubePort != None:
        cubePort.write(bytearray(packet,'utf-8'))