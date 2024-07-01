# -*- coding: utf-8 -*-
"""
Created on Mon Jul  1 17:16:06 2024

@author: eenmv
"""

import pixelFont
import numpy as np

import matplotlib.pyplot as plt



########################################
####### Virtual Light Cube #############
########################################


def virtualLightCube(LightCube,fig):
    """
    Returns an image of what the cube should look like.
    Some code suggested by ChatGPT :(

    Parameters
    ----------
    LightN : [i,j,k].
    LghtCube : np.array of cube state

    Returns
    -------
    Image: Image of a light cube

    """
    
    #fig = plt.figure()
    
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('grey')
    
    #Red
    x, y, z = np.where(LightCube[:,:,:,0] & 
                       np.bitwise_not(LightCube[:,:,:,1]) & 
                       np.bitwise_not(LightCube[:,:,:,2])
                       )
    ax.scatter(x, y, z, c='red', marker='o',alpha = 0.5)
    
    #Green
    x, y, z = np.where(np.bitwise_not(LightCube[:,:,:,0]) & 
                       LightCube[:,:,:,1] & 
                       np.bitwise_not(LightCube[:,:,:,2])
                       )
    ax.scatter(x, y, z, c='green', marker='o',alpha = 0.5)
    
    # Blue
    x, y, z = np.where(np.bitwise_not(LightCube[:,:,:,0]) & 
                       np.bitwise_not(LightCube[:,:,:,1]) & 
                       LightCube[:,:,:,2]
                       )
    ax.scatter(x, y, z, c='blue', marker='o',alpha = 0.5)
    
    # Yellow
    x, y, z = np.where(LightCube[:,:,:,0] & 
                       LightCube[:,:,:,1] & 
                       np.bitwise_not(LightCube[:,:,:,2])
                       )
    ax.scatter(x, y, z, c='yellow', marker='o',alpha = 0.5)
    
    # Magenta
    x, y, z = np.where(LightCube[:,:,:,0] & 
                       np.bitwise_not(LightCube[:,:,:,1]) & 
                       LightCube[:,:,:,2]
                       )
    ax.scatter(x, y, z, c='magenta', marker='o',alpha = 0.5)
    
    # Cyan
    x, y, z = np.where(np.bitwise_not(LightCube[:,:,:,0]) & 
                       LightCube[:,:,:,1] & 
                       LightCube[:,:,:,2]
                       )
    ax.scatter(x, y, z, c='cyan', marker='o',alpha = 0.5)
    
    # White
    x, y, z = np.where(LightCube[:,:,:,0] & 
                       LightCube[:,:,:,1] & 
                       LightCube[:,:,:,2]
                       )
    ax.scatter(x, y, z, c='white', marker='o',alpha = 0.5)
    
    ax.set_xlim([0,LightCube.shape[0]])
    ax.set_ylim([0,LightCube.shape[1]])
    ax.set_zlim([0,LightCube.shape[2]])
    
    return fig, ax



########################################
######## Generate Packets ##############
########################################


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


########################################
##########  Other rendering  ###########
########################################


def boundaryBox(LightCube, surfacelayer,SurfaceOnly):
    
    if SurfaceOnly:
        LightCube[:,:,surfacelayer,0] = True
        LightCube[:,:,surfacelayer,1] = True
        LightCube[:,:,surfacelayer,2] = False
    else:
        LightCube[:,:,surfacelayer,0] = True
        LightCube[:,:,surfacelayer,1] = True
        LightCube[:,:,surfacelayer,2] = False
        
        # LightCube[:,:,0,0] = True
        # LightCube[:,:,0,1] = True
        # LightCube[:,:,0,2] = False
        
        LightCube[0,0,:surfacelayer,0] = True
        LightCube[0,0,:surfacelayer,1] = True
        LightCube[0,0,:surfacelayer,2] = False
        
        LightCube[0,-1,:surfacelayer,0] = True
        LightCube[0,-1,:surfacelayer,1] = True
        LightCube[0,-1,:surfacelayer,2] = False
        
        LightCube[-1,0,:surfacelayer,0] = True
        LightCube[-1,0,:surfacelayer,1] = True
        LightCube[-1,0,:surfacelayer,2] = False
        
        LightCube[-1,-1,:surfacelayer,0] = True
        LightCube[-1,-1,:surfacelayer,1] = True
        LightCube[-1,-1,:surfacelayer,2] = False
        
        LightCube[:,0,0,0] = True
        LightCube[:,0,0,1] = True
        LightCube[:,0,0,2] = False
        
        LightCube[:,-1,0,0] = True
        LightCube[:,-1,0,1] = True
        LightCube[:,-1,0,2] = False
        
        LightCube[0,:,0,0] = True
        LightCube[0,:,0,1] = True
        LightCube[0,:,0,2] = False
        
        LightCube[-1,:,0,0] = True
        LightCube[-1,:,0,1] = True
        LightCube[-1,:,0,2] = False
    
    return LightCube


########################################
##########  Text Rendering  ############
########################################

def textDraw(Text,Col,LightCube,LightN,Pos,Scale):
    # Path: [0,0] -> [N,0] -> [N,N] -> [N,0] -> [0,0]
    Pathx = list(range(0,LightN[0]-1)) + (LightN[0]-1)*[LightN[0]-1] + list(range(LightN[0]-1,0,-1)) + (LightN[0]-1)*[0]
    Pathy = (LightN[0]-1)*[0] + list(range(0,LightN[0]-1)) + (LightN[0]-1)*[LightN[0]-1] + list(range(LightN[0]-1,0,-1))
    
    if LightN[0] != 16:
        temp = Pathy
        Pathy = Pathx
        Pathx = temp
    
    for Tn in range(len(Text)):
        Chr = Text[len(Text) - Tn - 1]
        FontMap = pixelFont.font8x8_basic[ord(Chr)]
        
        for i in range(int(8*Scale)):
            
            # Calculate position along path
            Pathn = (Tn * 9 * Scale + i ) + Pos
            
            if Pathn < 0 or Pathn > len(Pathx) - 1:
                continue
            
            Lx = Pathx[Pathn]
            Ly = Pathy[Pathn]
            
            for j in range(int(8*Scale)):
                
                fx = int(i / Scale)
                fy = int(j / Scale)
                
                LED = bool( FontMap[7-fy] & (1 << (7-fx)) ) #get i'th,j'th pixel
                
                LightCube[Lx,Ly,j,0] = LED & Col[0]
                LightCube[Lx,Ly,j,1] = LED & Col[1]
                LightCube[Lx,Ly,j,2] = LED & Col[2]
    
    
    return LightCube