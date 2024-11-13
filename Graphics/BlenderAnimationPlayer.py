# -*- coding: utf-8 -*-
"""
Created on Wed Nov 13 15:47:32 2024

@author: eenmv
"""

import numpy as np
import lightCubeUtil

import serial
import serial.tools.list_ports

import matplotlib.pyplot as plt


import time


# Load the comm port
try:
    Ports = serial.tools.list_ports.comports()
    if len(Ports)==0:
        raise
    print(*Ports)
    CommPortID = int(input("Select Port: "))
    
    cubePort = serial.Serial("COM"+str(CommPortID),115200)
    print("Connected succesfully")
    
except:
    
    try:
        import IPython
        shell = IPython.get_ipython()
        shell.enable_matplotlib(gui='qt')
    except:
        pass   
    
    CommPortID = None
    cubePort = None
    fig = plt.figure()
    print("Could not connect to THE CUBE, using virtual cube")



# Setup Light cube array
LightCube = np.zeros([16,16,16,3],dtype='bool')
LightCubeOld = np.copy(LightCube)

#Load the frames of the animation
Frames = lightCubeUtil.loadBlenderAnimation(r"C:\Users\eenmv\Documents\Github\NAMELightCube\Blender Animation Scene\temp1_LargeCube.txt")



# Play animation frame by frame
for F in range(100):
    
    # Clear cube because blenderDraw does clear before drawing
    LightCube = np.zeros([16,16,16,3],dtype = 'bool')
    LightCube = lightCubeUtil.blenderDraw(LightCube,Frames,F)

    #Generate the packet (string) to send to the pi pico, needs new and previous frame
    packet = lightCubeUtil.getUpdatedVoxels(LightCube, LightCubeOld)
    
    # Copy new to old frame
    LightCubeOld = np.copy(LightCube)
    
    # send data to THE CUBE if connected
    if cubePort != None:
        cubePort.write(bytearray(packet,'utf-8'))
    else:
        fig.clear()
        
        fig,ax = lightCubeUtil.virtualLightCube(LightCube,fig)
        
        plt.pause(0.05)
        
        fig.show()
        
    time.sleep(0.1)