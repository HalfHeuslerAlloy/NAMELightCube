# -*- coding: utf-8 -*-
"""
Created on Mon Jul  1 17:16:06 2024

@author: eenmv
"""

import pixelFont
import numpy as np

import matplotlib.pyplot as plt

from PIL import Image



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
    
    #fig.show()
    
    return fig, ax


def commandLineVLC(LightCube):
    fig = plt.figure()
    fig, ax = virtualLightCube(LightCube,fig)
    fig.show()


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
#######  Particle Rendering  ###########
########################################


def drawParticles(Particles,LightCube,LightN,DrawPriority):
    
    LightCube[:,:,:,:] = False #clear cube
    DrawPriority[:,:,:] = 0 #Clear Draw Priority
    
    for P in Particles:
        Pos = np.copy(P.Pos)
        Pos[0] = np.round(Pos[0]*LightN[0] - 0.5)
        Pos[1] = np.round(Pos[1]*LightN[1] - 0.5)
        Pos[2] = np.round(Pos[2]*LightN[2] - 0.5)
        
        i = int(Pos[0])
        j = int(Pos[1])
        k = int(Pos[2])
        
        # pShape = [[0,0,0],[0,0,1],[0,1,0],[0,1,1],
        #           [1,0,0],[1,0,1],[1,1,0],[1,1,1]]
        
        pShape = [[0,0,0]]
        
        if ( 0<=i<LightN[0] ) and ( 0<=j<LightN[1] ) and ( 0<=k<LightN[2] ):
            if DrawPriority[i,j,k] < P.DrawPri: #Check is the draw priority is larger
                if P.Col[0]>0.5:
                    for S in pShape:
                        LightCube[i+S[0],j+S[1],k+S[2],0] = True
                else:
                    for S in pShape:
                        LightCube[i+S[0],j+S[1],k+S[2],0] = False
                if P.Col[1]>0.5:
                    for S in pShape:
                        LightCube[i+S[0],j+S[1],k+S[2],1] = True
                else:
                    for S in pShape:
                        LightCube[i+S[0],j+S[1],k+S[2],1] = False
                if P.Col[2]>0.5:
                    for S in pShape:
                        LightCube[i+S[0],j+S[1],k+S[2],2] = True
                else:
                    for S in pShape:
                        LightCube[i+S[0],j+S[1],k+S[2],2] = False
            
                DrawPriority[i,j,k] = P.DrawPri
        
        # Draw any trails for the particle
        if P.Trail:
            N = len(P.PastPos)
            drawLineCube(P.Pos, P.PastPos[-1], P.Col, P.DrawPri)
            for n in range(N-1):
                drawLineCube(P.PastPos[n], P.PastPos[n+1], P.TrailCol, P.DrawPri)
    
    return LightCube


def drawLineCube(LightCube,LightN,DrawPriority,P1,P2,Col,DrawPri):
    
    N = int(np.linalg.norm(P2-P1)*max(LightN)*2) # Number of sample points
    if N==0:
        return
    for n in range(N+1):
        
        Pos = ( n/N * (P2-P1) + P1)
        
        Pos[0] = np.round(Pos[0]*LightN[0])
        Pos[1] = np.round(Pos[1]*LightN[1])
        Pos[2] = np.round(Pos[2]*LightN[2])
        
        i = int(Pos[0])
        j = int(Pos[1])
        k = int(Pos[2])
        
        #Check if in bounds
        if ( 0<=i<LightN[0] ) and ( 0<=j<LightN[1] ) and ( 0<=k<LightN[2] ):
            if DrawPriority[i,j,k] < DrawPri:
                if Col[0]>0.5:
                    LightCube[i,j,k,0] = True
                if Col[1]>0.5:
                    LightCube[i,j,k,1] = True
                if Col[2]>0.5:
                    LightCube[i,j,k,2] = True
            
                DrawPriority[i,j,k] = DrawPri
                
    return LightCube, DrawPriority


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

def CVSToImageArray(Filename):
    Arr = np.genfromtxt(Filename,dtype = 'int').astype('bool')
    Arr = Arr.transpose()
    ArrThird = int(Arr.shape[1]/3)
    ImgArr = np.zeros([ Arr.shape[0], ArrThird, 3],dtype = 'bool')
    
    ImgArr[:,:,0] = Arr[:,0:ArrThird]
    ImgArr[:,:,1] = Arr[:,ArrThird:ArrThird*2]
    ImgArr[:,:,2] = Arr[:,ArrThird*2:ArrThird*3]
    
    return ImgArr

def GIFToAnimateFrames(Filename):
    pass

def imageConverted(Img,Width,Height):
    """
    Parameters
    ----------
    image : image to to converted to a WxH pix and 3 bit color.

    Returns
    -------
    ImgArr

    """
    
    #Filename or PIL image
    if type(Img) == str:
        Img = Image.open( Img , "r")
        #Img = Img.rotate(90)
        Img.load()
        
    
    Data = np.asarray( Img, dtype="int32" )
    
    ImgArr = np.zeros([Width,Height,3],dtype ='bool')
    
    print(Data.shape)
    
    for W in range(Width):
        for H in range(Height):
            
            
            Rmean = np.mean( Data[ int(H*Data.shape[0]/Height):int((H+1)*Data.shape[0]/Height - 1),
                                   int(W*Data.shape[1]/Width):int((W+1)*Data.shape[1]/Width - 1),
                                   0] )
            Gmean = np.mean( Data[ int(H*Data.shape[0]/Height):int((H+1)*Data.shape[0]/Height - 1),
                                   int(W*Data.shape[1]/Width):int((W+1)*Data.shape[1]/Width - 1),
                                   1] )
            Bmean = np.mean( Data[ int(H*Data.shape[0]/Height):int((H+1)*Data.shape[0]/Height - 1),
                                   int(W*Data.shape[1]/Width):int((W+1)*Data.shape[1]/Width - 1),
                                   2] )
            
            ImgArr[W,H,0] = bool(round(Rmean/255))
            ImgArr[W,H,1] = bool(round(Gmean/255))
            ImgArr[W,H,2] = bool(round(Bmean/255))
    
    
    
    return ImgArr

def imageDrawPerimeter(LightCube, ImgArr, Pos, Scale):
    
    LightN = LightCube.shape
    
    Pathx = list(range(0,LightN[0]-1)) + (LightN[0]-1)*[LightN[0]-1] + list(range(LightN[0]-1,0,-1)) + (LightN[0]-1)*[0]
    Pathy = (LightN[0]-1)*[0] + list(range(0,LightN[0]-1)) + (LightN[0]-1)*[LightN[0]-1] + list(range(LightN[0]-1,0,-1))
    
    # 16x16x16 mirrored?
    if LightN[0] != 16:
        temp = Pathy
        Pathy = Pathx
        Pathx = temp
    
    for i in range(len(Pathx)):
        Imgx = ImgArr.shape[0] - int( (i-Pos[0])/Scale)
        
        if Imgx<0 or Imgx>ImgArr.shape[0]-1:
            continue
        
        Lx = Pathx[i]
        Ly = Pathy[i] 
        
        for j in range(LightN[2]):
            
            Imgy = ImgArr.shape[1] - int( (j-Pos[1])/Scale)
            
            if Imgy<0 or Imgy>ImgArr.shape[1]-1:
                continue
            
            LightCube[Lx,Ly,j,0] = ImgArr[Imgx,Imgy,0]
            LightCube[Lx,Ly,j,1] = ImgArr[Imgx,Imgy,1]
            LightCube[Lx,Ly,j,2] = ImgArr[Imgx,Imgy,2]

    return LightCube
            
    

########################################
##########  Text Rendering  ############
########################################

def textDraw(LightCube,Text,Col,Pos,Scale):
    # Path: [0,0] -> [N,0] -> [N,N] -> [N,0] -> [0,0]
    LightN = LightCube.shape
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
            Pathn = (Tn * 9 * Scale + i ) + Pos[0]
            
            if Pathn < 0 or Pathn > len(Pathx) - 1:
                continue
            
            Lx = Pathx[Pathn]
            Ly = Pathy[Pathn]
            
            for j in range(int(8*Scale)):
                
                fx = int(i / Scale)
                fy = int(j / Scale)
                
                LED = bool( FontMap[7-fy] & (1 << (7-fx)) ) #get i'th,j'th pixel
                
                LightCube[Lx,Ly,j+Pos[1],0] = LED & Col[0]
                LightCube[Lx,Ly,j+Pos[1],1] = LED & Col[1]
                LightCube[Lx,Ly,j+Pos[1],2] = LED & Col[2]
    
    
    return LightCube