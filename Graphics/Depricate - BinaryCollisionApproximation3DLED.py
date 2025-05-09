import serial
import serial.tools.list_ports

import numpy as np
import tkinter as tk
from tkinter import font as tkFont

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import time

import winsound
import os
import pygame


# Table from github repo, https://gist.github.com/GoodmanSciences/c2dd862cd38f21b0ad36b8f96b4bf1ee
PeriodicTablefFile = "Periodic Table of Elements.csv"

def loadTable(file):
    """
    Takes in file outputs ordered list of elements
    """
    
    Elements = []
    
    with open(file,"r") as f:     
        for line in f.readlines():
            
            line.strip("\n")
            
            info = line.split(",")
            
            Elements.append(info)
    
    return Elements


try:
    pygame.mixer.init()
    pygame.mixer.set_num_channels(20)
    
    
    #Sound1 = pygame.mixer.Sound("07043286_shortv2.wav")
    Sound1 = pygame.mixer.Sound("Chirp0.1ms.wav")
    Sound2 = pygame.mixer.Sound("Chirp0.1msAlto.wav")
except:
    print("Failed to start audio mixer")
    Sound1 = None
    Sound2 = None

MinDelay = 0.01


# cube is always 1 unit
LightN = [3*8,3*8,32]
LightCube = np.zeros([LightN[0],LightN[1],LightN[2],3],dtype="bool") # cube N*N*N*3 RGB
DrawPriority = np.zeros([LightN[0],LightN[1],LightN[2]],dtype="byte")

#Time step
dT = 0.5

#GUI disable graphics
UpdateCanvas = True

cubePort = None



# Setup simulation configuration
SimConfig = {}

SimConfig["Ion_N"] = 1
SimConfig["Ion_Mass"] = 2
SimConfig["Ion_Position"] = np.array([0.5,0.5,0.95])
SimConfig["Ion_Velocity"] = np.array([0,0,-1])

SimConfig["Film_Thickness"] = 0.75
SimConfig["Film_Mass"] = 29
SimConfig["Film_N_Density"] = 50
SimConfig["Film_SecondaryThreshold"] = 25
SimConfig["Film_StickThreshold"] = 0.15


# Base particle class
class particle:
    def __init__(self,Pos,Vel,Mass=1,Charge=1,Col=[1,0,0],DrawPri=1,ID="None"):
        
        #Might be useful later???
        self.ID = ID
        self.LineIDs = []
        #inital condition
        self.Pos = Pos
        self.Vel = Vel
        self.DisSinceLastCollision = 0
        self.Mass = Mass
        self.Charge = Charge
        #colour
        self.Col = Col
        #draw priority, high more important
        # Intial ion should have larger priority for example
        self.DrawPri = DrawPri
        self.Trail = False
        self.TrailCol = self.Col
        self.PastPos = [Pos]
    
    def move(self,ThinFilm,dT):
        
        E = np.linalg.norm(self.Vel)**2*self.Mass*0.5 # Calculate kinetic energy
        
        InFilm = (self.Pos[2]<ThinFilm.Thk)
        
        Dis = np.linalg.norm(self.Vel)*dT
        
        # Film is sticky and below threshold strongly damp velocity
        # if (E < ThinFilm.StickThreshold) & InFilm:
        #     self.Vel = self.Vel*0.1
        
        #Electric stopping
        if InFilm and E != 0:
            E_lost = np.clip(np.power(E,0.5) * 10 * Dis,0,E)
            
            fraction = np.sqrt((E-E_lost)*2/self.Mass) / np.linalg.norm(self.Vel)
            
            self.Vel = self.Vel * fraction
        
        self.Pos = self.Pos + dT*self.Vel
        
        if InFilm:
            self.DisSinceLastCollision += Dis
        
        #Probablity of Collision
        
        if self.DisSinceLastCollision > ThinFilm.N_Den:
            #Check if in thin film, 50% chance of collision per N_Den distance
            if ( 0.5 > np.random.random() ):
                #inside film, do the 'monto carlo'
                
                # Only store collisions points
                if self.Trail:
                    self.PastPos.append(self.Pos)
                
                #get unit velocity vector of particle
                UnitVel = self.Vel/np.linalg.norm(self.Vel)
                
                #pick random point in space, as contact vector
                ConVec = np.random.random(3)*2-1
                ConVec = ConVec/np.linalg.norm(ConVec) #normalize
                DotProd = np.dot(ConVec,UnitVel)
                
    
                if DotProd<0:#mirror for only forward facing collisions
                    ConVec = ConVec - 2*UnitVel*DotProd
                
                
                IntDotVel = np.dot(ConVec,self.Vel)
                
                # calculate momentum exchange
                FinalDotVel = ( (self.Mass-ThinFilm.Mass)/(self.Mass+ThinFilm.Mass) ) * IntDotVel
                
                self.Vel = self.Vel + (FinalDotVel-IntDotVel)*ConVec
                
                #Return momentum exchange for particle generation
                return self.Mass*(FinalDotVel-IntDotVel)*ConVec
            else:
                self.DisSinceLastCollision = 0
        return np.array([0,0,0])


# Base thin film class
class thinFilm:
    def __init__(self,Thk = 0.75, Mass = 1, N_Den = 1, SecondThreshold = 0.5, StickThreshold = 0.01):
        self.Thk = Thk
        self.Mass = Mass #mean mass of thin film particles
        self.N_Den = 1/N_Den #mean distance between collision
        self.SecondThreshold = SecondThreshold
        self.StickThreshold = StickThreshold


# Main GUI window to display animation, outputs lightcube data at end
# TODO convert canvas to matplotlib 3d plotting instead.
class Window(tk.Frame):
    """
    Main window for AutoLab
    This class should handle GUI
    """
    
    InfoTemplate = """Current Setup:         
        Speed (Km/sec) : {}
        Ion: {}
        Atomic Number: {}
        Atomic Mass: {}"""
    
    def __init__(self, master):
        """
        Initial setup of widgets and the general window position
        """
        
        global SimConfig, cubePort
        
        self.master = master
        
        super().__init__(master)
        
        self.Size = 400
        
        self.Angle = 0
        self.PerspectiveAngle = np.pi / 4  # Perspective angle (45 degrees)
        self.Distance = 10  # Initial distance for zoom
        self.Azimuth = 30
        self.Elevation = 20
        
        self.TimeSinceLastSound = time.perf_counter()
        
        self.SimRunning = False
        
        # Initialize matplotlib figure
        self.fig = plt.figure(figsize=(12, 6))
        
        # Particle simulation plot
        self.ax1 = self.fig.add_subplot(121, projection='3d')
        
        # LED cube plot
        self.ax2 = self.fig.add_subplot(122, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(column=0, row=0, rowspan=10, columnspan=2)
        
        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.canvas.mpl_connect('button_press_event', self.on_press)
        self.canvas.mpl_connect('button_release_event', self.on_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_motion)
        
        #####  Control buttons #####
        
        ControlFrame = tk.Frame(master)
        ControlFrame.grid(column = 2, row = 0, rowspan=10)
        
        self.FireButton = tk.Button(ControlFrame,
                                         text = "Fire",
                                         bg = "red",
                                         command = self.fireSim,
                                         font=tkFont.Font(size=30)
                                         )
        self.FireButton.grid(column = 1, row = 2,padx=20)
        
        self.HaltButton = tk.Button(ControlFrame,
                                         text = "Halt",
                                         bg = "orange",
                                         command = self.haltSim,
                                         font=tkFont.Font(size=30)
                                         )
        self.HaltButton.grid(column = 2, row = 2,padx=20)
        
        self.ClearButton = tk.Button(ControlFrame,
                                         text = "Clear",
                                         bg = "green",
                                         command = self.clearSim,
                                         font=tkFont.Font(size=30)
                                         )
        self.ClearButton.grid(column = 3, row = 2,padx = 20)
        
        self.ResumeButton = tk.Button(ControlFrame,
                                         text = "Resume",
                                         bg = "blue",
                                         command = self.resumeSim,
                                         font=tkFont.Font(size=30)
                                         )
        self.ResumeButton.grid(column = 4, row = 2, padx = 20)
        
        SpeedInputText = tk.Label(ControlFrame,text = "Speed")
        SpeedInputText.grid(column=1,row = 0)
        self.SpeedInput = tk.DoubleVar() 
        self.SpeedScale = tk.Scale(ControlFrame,
                                   variable=self.SpeedInput,
                                   from_=99,
                                   to=10,
                                   orient=tk.VERTICAL,
                                   showvalue = False,
                                   length = 200,
                                   width = 45)  
        self.SpeedScale.grid(column = 1, row = 1)
        
        
        MassInputText = tk.Label(ControlFrame,text = "Mass")
        MassInputText.grid(column=2,row = 0)
        self.MassInput = tk.DoubleVar() 
        self.MassScale = tk.Scale(ControlFrame,
                                  variable=self.MassInput,
                                  from_=118,
                                  to=1,
                                  orient=tk.VERTICAL,
                                  showvalue = False,
                                  length = 200,
                                  width = 45)  
        self.MassScale.grid(column = 2, row = 1)
        
        
        self.SetupLabel = tk.Label(ControlFrame,text = self.InfoTemplate)
        self.SetupLabel.grid(column = 3, row = 1)
        self.GetSetupInfo()
        
        self.PreviosTime = time.time()
        self.Steps = 0
        
        self.dragging = False
        self.prev_mouse_x = None
        self.prev_mouse_y = None
        
        self.update()
        
    
    def generateSim(self):
        
        global dT,Elements
        
        self.Film = thinFilm(SimConfig["Film_Thickness"],
                             SimConfig["Film_Mass"],
                             SimConfig["Film_N_Density"],
                             SimConfig["Film_SecondaryThreshold"],
                             SimConfig["Film_StickThreshold"])
        
        self.Particles = []
        
        Mass = self.MassInput.get()
        
        Mass = float(Elements[int(Mass)][3])
        
        Vel = np.sqrt( self.SpeedInput.get() * 100 ) / Mass * np.array([0,0,-0.1])
        
        Vel = 0.1*self.SpeedInput.get() * np.array([0,0,-0.1])
        
        print(Vel)
        
        dT = 0.05 / abs(Vel[2])
        
        #dT = 0.025
        
        totalEnergy = 0
        
        for I in range(SimConfig["Ion_N"]):
            
            RanPos = np.array([np.random.random()-0.5,
                               np.random.random()-0.5,
                               I*Vel[2]*dT*5])
            

            Ion = particle(SimConfig["Ion_Position"] + RanPos*0.1,
                           Vel,
                           Mass,
                           Col = [1,1,1])
            
            Ion.Trail = False
            Ion.TrailCol = [1,0,0]
            Ion.DrawPri = 2
            
            totalEnergy += Vel[2]**2 * Mass * 0.5
            
            self.Particles.append(Ion)
        
        
        self.Film.SecondThreshold = abs(Vel[2]) * Mass / 15
        
        self.Film.SecondThreshold = 0.025
        
        self.drawParticles()
    
    
    def fireSim(self):
        
        self.clearSim()
        
        self.generateSim()
        
        self.SimRunning = True
        print("Firing Ions")
    
    def haltSim(self):
        
        self.SimRunning = False
        print("Stop Simulation")
    
    def resumeSim(self):
        
        self.SimRunning = True
        print("Resuming Simulation")
    
    def clearSim(self):
        
        self.haltSim()
        
        self.ax1.clear()
        self.ax2.clear()
        
        self.Particles = []
        
        #Update cube display
        self.outputCube()
    
    def GetSetupInfo(self):
        
        global Elements
        
        Vel = 0.1*self.SpeedInput.get() * np.array([0,0,-0.1])
        
        N = self.MassInput.get()
        
        Element = Elements[int(N)]
        
        self.SetupLabel["text"] = self.InfoTemplate.format(round(Vel[2]*-520),
                                                            Element[1],
                                                            Element[0],
                                                            Element[3]
                                                            )
        
    
    def update(self):
        global dT, UpdateCanvas, cubePort, LightCube
        
        NewParticles = []
        
        self.GetSetupInfo()
        
        self.Angle = self.Angle + 0.01
        
        Energy = 0
        
        if self.SimRunning:
            for P in self.Particles:
                
                M = P.move(self.Film,dT)
                
                if type(M) != None:
                    
                    MagM = np.linalg.norm(M) * 0.5
                    
                    global Sound1, Sound2, MinDelay
                    
                    
                    # Check if the mixer startup worked or has a valid audio out
                    if Sound1 != None:
                        try:
                            SoundVolume = np.clip(np.sqrt(MagM)/10,0,0.5)
                            if time.perf_counter() - self.TimeSinceLastSound > MinDelay and SoundVolume:
                                chan = pygame.mixer.find_channel()
                                chan.set_volume(SoundVolume,SoundVolume)
                                if 0.5>np.random.random():
                                    chan.play(Sound1)
                                else:
                                    chan.play(Sound2)
                                self.TimeSinceLastSound = time.perf_counter() + np.random.random()*MinDelay
                        except:
                            pass

                    
                    # if MagM > 0:
                    #     CollisionPoint = particle(P.Pos, np.zeros(3),Mass=self.Film.Mass,Col=[0,0,1])
                    #     NewParticles.append(CollisionPoint)
                    
                    if len(self.Particles)<500:
                        if MagM > self.Film.SecondThreshold:
                            
                            #limit density of new particles
                            MinDis = 999
                            
                            for NewP in NewParticles:
                                MinDis = min(MinDis, np.linalg.norm(P.Pos-NewP.Pos))
                                
                            for NewP in self.Particles[25:]:
                                MinDis = min(MinDis, np.linalg.norm(P.Pos-NewP.Pos))
                            
                            # Don't generate secondaries close together
                            if MinDis > 0.01 and len(NewParticles)<25:
                                Secondary = particle(P.Pos, -M/self.Film.Mass,Mass=self.Film.Mass,Col=[1,0,0])
                                NewParticles.append(Secondary)
                    
                    Energy += np.linalg.norm(P.Vel)**2*P.Mass*0.5
            
            #print("Total Energy: {}".format(Energy))
            
            self.outputCube()
            
            if cubePort != None:
                Send(cubePort,LightCube)
            
            for P in NewParticles:
                self.Particles.append(P)
            
            if UpdateCanvas:
                self.drawParticles()
                self.drawLEDs()
        
            
        self.after(5,self.update)
    
    def drawParticles(self):
        self.ax1.clear()
        
        xs = [P.Pos[0] for P in self.Particles]
        ys = [P.Pos[1] for P in self.Particles]
        zs = [P.Pos[2] for P in self.Particles]
        colors = [self.convertColour(P.Col) for P in self.Particles]
        
        # Plot black outlines
        self.ax1.scatter(xs, ys, zs, c='black', s=80, edgecolors='black')
        # Plot actual colors
        self.ax1.scatter(xs, ys, zs, c=colors, s=50, edgecolors='black')
        
        self.ax1.set_xlim(0, 1)
        self.ax1.set_ylim(0, 1)
        self.ax1.set_zlim(0, 1)
        
        self.ax1.view_init(elev=self.Elevation, azim=self.Azimuth)
        self.ax1.dist = self.Distance  # Set initial distance
        
        self.canvas.draw()

    def drawLEDs(self):
        self.ax2.clear()

        x, y, z = LightCube[:, :, :, 0].nonzero()
        self.ax2.scatter(x, y, z, c='red', label='Red LEDs')

        x, y, z = LightCube[:, :, :, 1].nonzero()
        self.ax2.scatter(x, y, z, c='green', label='Green LEDs')

        x, y, z = LightCube[:, :, :, 2].nonzero()
        self.ax2.scatter(x, y, z, c='blue', label='Blue LEDs')

        self.ax2.set_xlim(0, LightN[0])
        self.ax2.set_ylim(0, LightN[1])
        self.ax2.set_zlim(0, LightN[2])

        self.ax2.view_init(elev=self.Elevation, azim=self.Azimuth)
        self.ax2.dist = self.Distance  # Set initial distance

        self.ax2.legend()
        self.canvas.draw()
    
    def convertColour(self,Col):
        
        Str = "#"
        Str=Str+"{:02x}".format(round(Col[0]*255))
        Str=Str+"{:02x}".format(round(Col[1]*255))
        Str=Str+"{:02x}".format(round(Col[2]*255))

            
        return Str
    
    def on_scroll(self, event):
        if event.button == 'up':
            self.Distance = max(1, self.Distance - 1)
        elif event.button == 'down':
            self.Distance += 1
        self.update_view()
    
    def on_press(self, event):
        if event.button == 1:
            self.dragging = True
            self.prev_mouse_x = event.x
            self.prev_mouse_y = event.y
    
    def on_release(self, event):
        if event.button == 1:
            self.dragging = False
            self.prev_mouse_x = None
            self.prev_mouse_y = None
    
    def on_motion(self, event):
        if self.dragging:
            dx = event.x - self.prev_mouse_x
            dy = event.y - self.prev_mouse_y
            self.prev_mouse_x = event.x
            self.prev_mouse_y = event.y
            self.Azimuth += dx * 0.1
            self.Elevation -= dy * 0.1
            self.update_view()
    
    def update_view(self):
        self.ax1.view_init(elev=self.Elevation, azim=self.Azimuth)
        self.ax1.dist = self.Distance
        self.ax2.view_init(elev=self.Elevation, azim=self.Azimuth)
        self.ax2.dist = self.Distance
        self.canvas.draw()
    
    def outputCube(self):
        
        global LightCube,LightN,DrawPriority,SimConfig
        
        Start = time.perf_counter()
        
        LightCube[:,:,:,:] = False #clear cube
        DrawPriority[:,:,:] = 0 #Clear Draw Priority
        
        surfacelayer = int( SimConfig["Film_Thickness"] * LightN[2] )
        
        LightCube[:,:,surfacelayer,0] = True
        LightCube[:,:,surfacelayer,1] = True
        LightCube[:,:,surfacelayer,2] = False
        
        for P in self.Particles:
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
                    if P.Col[1]>0.5:
                        for S in pShape:
                            LightCube[i+S[0],j+S[1],k+S[2],1] = True
                    if P.Col[2]>0.5:
                        for S in pShape:
                            LightCube[i+S[0],j+S[1],k+S[2],2] = True
                
                    DrawPriority[i,j,k] = P.DrawPri
            
            # Draw any trails for the particle
            if P.Trail:
                N = len(P.PastPos)
                self.drawLineCube(P.Pos, P.PastPos[-1], P.Col, P.DrawPri)
                for n in range(N-1):
                    self.drawLineCube(P.PastPos[n], P.PastPos[n+1], P.TrailCol, P.DrawPri)

        End = time.perf_counter()
        
        #print("Seconds to draw cube: {}".format((End-Start)))
    
    
    def drawLineCube(self,P1,P2,Col,DrawPri):
        
        global LightCube,LightN,DrawPriority
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
    
    def plotCube(self):
        
        global LightCube
        
        voxelarray = LightCube[:,:,:,0] | LightCube[:,:,:,1] | LightCube[:,:,:,2]
        
        colors = np.empty(voxelarray.shape, dtype=object)
        colors[LightCube[:,:,:,0]] = 'red'
        colors[LightCube[:,:,:,1]] = 'green'
        colors[LightCube[:,:,:,2]] = 'blue'

        # and plot everything
        ax = plt.figure().add_subplot(projection='3d')
        ax.voxels(voxelarray, facecolors=colors)

        plt.show()
            

def Send(port,lightCube):
    
    framepacket = chr(0b10000000) # start with the reset character
    
    for k in range(32):
        for j in range(8):
            for i in range(4):
                
                char = ( 0b01000000 |
                        lightCube[2*i,j,k,0] << 0 |
                        lightCube[2*i,j,k,1] << 1 |
                        lightCube[2*i,j,k,2] << 2 |
                        lightCube[2*i+1,j,k,0] << 3 |
                        lightCube[2*i+1,j,k,1] << 4 |
                        lightCube[2*i+1,j,k,2] << 5
                        )
                
                framepacket += chr(char)
    
    
    port.write(bytearray(framepacket,'utf-8'))
    

if __name__=="__main__":
    
    try:
        Ports = serial.tools.list_ports.comports()
        print("Ports:")
        for p in range(len(Ports)):
            print(Ports[p].device)
        Selection = int(input("Select Port: "))
        cubePort = serial.Serial("COM"+str(Selection),115200)
    except:
        print("Could not connect to THE CUBE")
        
    #load the elements
    Elements = loadTable(PeriodicTablefFile)
    print("Loaded the elements")
    
    #Make and start main window
    root = tk.Tk()
    Sim = Window(root)
    root.title("Cube display")
    Sim.grid(column=0, row=0, sticky="nsew")

    root.mainloop()
    
    cubePort.close()
    print("close port")
