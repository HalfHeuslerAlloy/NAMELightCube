# -*- coding: utf-8 -*-
"""
Created on Thu May 23 11:22:16 2024

@author: eenmv
"""

import numpy as np
import time
import serial
import serial.tools.list_ports

import pixelFont

import matplotlib.pyplot as plt

packet = ""

cubeSize = [24,24,32]

cube = None
oldCube = None
cubePort = None

def getUpdatedVoxels(NewFrame,OldFrame):
    Diff = np.bitwise_xor(NewFrame,OldFrame)
    DiffPos = np.nonzero(Diff)
    UpdatePacket = ""
    
    i_last = -1
    j_last = -1
    k_last = -1
    
    for Pn in range(len(DiffPos[0])):
        
        i = DiffPos[0][Pn]
        j = DiffPos[1][Pn]
        k = DiffPos[2][Pn]
        
        if i_last == i and j_last == j  and k_last == k:
            continue
        else:
            
            i_last = i
            j_last = j
            k_last = k
            
            UpdatePacket += chr(0b01000000 |
                                NewFrame[i,j,k,0] << 5 |
                                i)
            UpdatePacket += chr(0b00000000 |
                                NewFrame[i,j,k,1] << 5 |
                                j)
            UpdatePacket += chr(0b00000000 |
                                NewFrame[i,j,k,2] << 5 |
                                k)
    return UpdatePacket


def updateCube():
    global cube, oldCube, cubePort
    packet = getUpdatedVoxels(cube, oldCube)
    if cubePort != None:
        cubePort.write(bytearray(packet,'utf-8'))
    
    
def testSpeed(commPortID):
    
    global packet,cubeSize
    global cube, oldCube, cubePort
    
    try:
        cubePort = serial.Serial("COM"+str(CommPortID),115200)
        print("connected to cube")
    except:
        cubePort = None
        print("Failed to connect to cube")
    
    cube = np.zeros([cubeSize[0],cubeSize[1],cubeSize[2],3],dtype='bool')
    oldCube = np.zeros([cubeSize[0],cubeSize[1],cubeSize[2],3],dtype='bool')
    
    N = 1
    
    gridPattern = np.array([[1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0],
                            [1,0,0,0,1,0,0,0,1,0,1,1,1,0,0,0,1,0,0,1,1,1,0,0],
                            [0,0,0,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0],
                            [0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,1,0,0],
                            [0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
                            [0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0],
                            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                            [1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0],
                            [1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],
                            [0,0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0],
                            [0,0,1,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0],
                            [0,0,1,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0],
                            [0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,1,0,0],
                            [0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0],
                            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                            [1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0],
                            [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],
                            [0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0],
                            [0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,1,0,1,0,0],
                            [0,0,0,0,0,1,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0],
                            [0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,1,0,0],
                            [0,0,0,0,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0,1,1,1,0,0],
                            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                            ],
                           dtype='bool')
    
    Col = 0
    
    while(True):
        
        Str = input("next line: ")
        
        if len(Str) > 0:
            N = 0
            Col = 0
            cube[:,:,:,:] = False
            textScroll(Str,[1,0,0],cube,cubeSize,1)
        else:
            cube[:,:,:,:] = False
            if cubeSize[0] == 24:
                cube[:,:,0,0] = gridPattern
            cube[:,:,N,Col] = True
            
            N += 1
            
            if N>cubeSize[2]-1:
                N = 0
                Col = (Col + 1)%3
            
        
        start = time.perf_counter()
        
        packet = getUpdatedVoxels(cube, oldCube)
        
        stop = time.perf_counter()
        
        print("Packet Generation Time {}".format(stop-start))
        
        start = time.perf_counter()
        
        if cubePort != None:
            cubePort.write(bytearray(packet,'utf-8'))
        
        stop = time.perf_counter()
        
        print("Packet Send Time {}".format(stop-start))
        
        oldCube = np.copy(cube)


def textScroll(Text,Colour,LightCube,LightN,Scale):
    global cube, oldCube, cubePort
    for Pos in range(-10*len(Text)*Scale,LightN[0]*4):
        cube = textDraw(Text,[1,0,0],LightCube,[16,16,16],Pos,1)
        updateCube()
        time.sleep(0.05)
        oldCube = np.copy(cube)

        


def textDraw(Text,Colour,LightCube,LightN,Pos,Scale):
    # Path: [0,0] -> [N,0] -> [N,N] -> [N,0] -> [0,0]
    Pathx = list(range(0,LightN[0]-1)) + (LightN[0]-1)*[LightN[0]-1] + list(range(LightN[0]-1,0,-1)) + (LightN[0]-1)*[0]
    Pathy = (LightN[0]-1)*[0] + list(range(0,LightN[0]-1)) + (LightN[0]-1)*[LightN[0]-1] + list(range(LightN[0]-1,0,-1))
    
    for Tn in range(len(Text)):
        Chr = Text[len(Text) - Tn - 1]
        FontMap = pixelFont.font8x8_basic[ord(Chr)]
        
        for i in range(8):
            
            # Calculate position along path
            Pathn = (Tn * 10 + i ) * Scale + Pos
            
            if Pathn < 0 or Pathn > len(Pathx) - 1:
                continue
            
            Lx = Pathx[Pathn]
            Ly = Pathy[Pathn]
            
            for j in range(8):
                LED = bool( FontMap[7-j] & (1 << (7-i)) ) #get i'th,j'th pixel
                
                LightCube[Lx,Ly,j*Scale,0] = LED & Colour[0]
                LightCube[Lx,Ly,j*Scale,1] = LED & Colour[1]
                LightCube[Lx,Ly,j*Scale,1] = LED & Colour[2]
    
    
    return LightCube

def plotCube(LightCube):
    
    voxelarray = LightCube[:,:,:,0] | LightCube[:,:,:,1] | LightCube[:,:,:,2]
    
    colors = np.empty(voxelarray.shape, dtype=object)
    colors[LightCube[:,:,:,0]] = 'red'
    colors[LightCube[:,:,:,1]] = 'green'
    colors[LightCube[:,:,:,2]] = 'blue'

    # and plot everything
    ax = plt.figure().add_subplot(projection='3d')
    ax.voxels(voxelarray, facecolors=colors)

    plt.show()
                
        

if __name__=="__main__":
    try:
        Ports = serial.tools.list_ports.comports()
        if len(Ports)==0:
            raise
        print(*Ports)
        CommPortID = int(input("Select Port: "))
    except:
        CommPortID = None
        print("Could not connect to THE CUBE")
    
    try:
        sizeSelection = int(input("Select Size: 1:{8,8,32}, 2:{16,16,16}, 3:{24,24,32} : "))
        
        sizes = [[8,8,32],[16,16,16],[24,24,32]]
        
        cubeSize = sizes[sizeSelection - 1]
        
    except:
        print("Failed to select cube size")
        raise
    
    
    testSpeed(CommPortID)
                    