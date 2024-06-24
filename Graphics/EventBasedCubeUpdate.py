# -*- coding: utf-8 -*-
"""
Created on Thu May 23 11:22:16 2024

@author: eenmv
"""

import numpy as np
import time
import serial
import serial.tools.list_ports

packet = ""

cubeSize = [24,24,32]

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
    
def testSpeed(commPortID):
    
    global packet,cubeSize
    
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
        cube[:,:,:,:] = False
        if cubeSize[0] == 24:
            cube[:,:,0,0] = gridPattern
        cube[:,:,N,Col] = True
        
        start = time.perf_counter()
        
        packet = getUpdatedVoxels(cube, oldCube)
        
        stop = time.perf_counter()
        
        print("Packet Generation Time {}".format(stop-start))
        
        start = time.perf_counter()
        
        if cubePort != None:
            cubePort.write(bytearray(packet,'utf-8'))
        
        stop = time.perf_counter()
        
        print("Packet Send Time {}".format(stop-start))
        
        N += 1
        
        if N>cubeSize[2]-1:
            N = 0
            Col = (Col + 1)%3
        
        Str = input("next line")
        
        oldCube = np.copy(cube)

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
                    