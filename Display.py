import sys
import pygame
import math
import os
from pygame.locals import *
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
from Text import getTextSurface

class Display:
    def __init__(self,blimpHandler):
        self.activeController = 0
        #Store BlimpHandler parameter
        self.blimpHandler = blimpHandler
        blimpHandler.setDisplay(self)
        self.alive = True

        # Anchors
        self.width_screen = 1000
        self.height_screen = 600
        self.anchor_x_inputVisual = 0
        self.anchor_y_inputVisual = 0
        self.width_inputVisual = 200
        self.anchor_x_inputVisualRect = self.anchor_x_inputVisual + 20
        self.anchor_y_inputVisualRect = self.anchor_y_inputVisual + 35
        self.width_inputVisualRect = 160
        self.height_inputVisualRect = 80
        self.inputVisualRect_radius = 35
        self.inputVisualRect_JSCircle = 5
        self.anchor_x_input = self.anchor_x_inputVisual + self.width_inputVisual
        self.anchor_y_input = self.anchor_y_inputVisual
        self.width_input = 225
        self.align_input_right = self.anchor_x_input+120
        self.anchor_y_inputText = 60
        self.spacing_y_inputText = 100
        self.anchor_x_blimps = self.anchor_x_input + self.width_input
        self.anchor_y_blimps = 0
        self.align_blimps_left = self.anchor_x_blimps
        self.width_blimps = 250
        self.anchor_y_blimpText = 60
        self.spacing_y_blimpText = 45
        self.anchor_x_MPB = self.anchor_x_blimps + 125
        self.anchor_y_MPB = 66
        self.spacing_y_MPB = self.spacing_y_inputText
        self.anchor_x_activeController = self.anchor_x_blimps + self.width_blimps
        self.anchor_y_activeController = 0
        self.anchor_x_activeControllerIndicator = self.anchor_x_input + 10
        self.anchor_y_activeControllerIndicator = 73
        self.anchor_x_blimpState = self.anchor_x_blimps + self.width_blimps
        self.anchor_y_blimpState = 15
        self.width_blimpState = 200
        self.anchor_x_blimpStatus = self.anchor_x_blimpState + self.width_blimpState
        self.anchor_y_blimpStatus = self.anchor_y_blimpState
        self.width_blimpStatus = 200

        self.anchor_x_blimpStateLegend = self.width_screen - 200
        self.anchor_y_blimpStateLegend = self.height_screen - 100
        self.spacing_y_blimpStateLegend = 50

        self.anchor_x_barometer = 50
        self.anchor_y_barometer = self.height_screen - 50

        #Colors
        self.activeColor = Color(0,255,255)
        self.color_inputVisual_background = Color(100,100,100)
        self.color_inputVisual_grid = Color(150,150,150)
        self.color_inputVisual_joystick = Color(255,255,255)
        self.color_blimpState_autonomous = Color(0,255,0)
        self.color_blimpState_manual = Color(0,0,0)

        # Game Display
        print("Beginning Program.")
        self.screen = pygame.display.set_mode((self.width_screen, self.height_screen))
        print("Size:",self.screen.get_size())
        pygame.display.set_caption("Multi-Blimp LTA Control")

        # Set up screen
        self.background = pygame.Surface(self.screen.get_size())
        self.background = self.background.convert()
        self.background.fill((200, 200, 200))
        self.screen.blit(self.background, (0, 0))

        #Set up variables
        self.textSurfaces = {};

        self.textSurface_Input = getTextSurface("Input:",50)
        self.textSurface_Blimps = getTextSurface("Blimps:",50)
        self.textSurface_State = getTextSurface("State",30)
        self.textSurface_Status = getTextSurface("Status",30)
        #self.textSurface_Clamp = getTextSurface("Clamp",30)
        self.textSurface_Record = getTextSurface("\"r\" to record",30)
        self.textSurface_Panic = getTextSurface("\"p\" for autonomous panic",30)

        #Multi-Purpose Button (MPB)
        self.MPBSurface_Enabled = pygame.Surface((15,15))
        self.MPBSurface_Enabled = self.MPBSurface_Enabled.convert()
        self.MPBSurface_Enabled.fill((200,0,200))
        self.MPBSurface_Disabled = pygame.Surface((15,15))
        self.MPBSurface_Disabled = self.MPBSurface_Disabled.convert()
        self.MPBSurface_Disabled.fill((0,255,0))
        self.buttons = []
        #self.buttons.append(((0,0),self.textSurface_Input.get_size(),"InputLabel"))
        self.drawing = False

        #Create state surfaces
        stateStringMap = blimpHandler.blimpStateStrings
        self.stateSurfaceMap = {}
        for key in stateStringMap.keys():
            self.stateSurfaceMap[key] = getTextSurface(stateStringMap[key],30)

        print("Display Initialized")

        self.exclusiveConnections = True

    """ #Hotfix add keybind
    elif self.getKey(K_e):
        if (self.activeController != -1):
            blimpIDs = []
            for connection in self.blimpHandler.connections:
                if (self.activeController == connection.inputIndex):
                    blimpIDs.append(self.blimpHandler.blimps[connection.blimpIndex].ID)
            if (len(blimpIDs) > 0):
                pass #self.blimpHandler.updateGrabber(blimpIDs)
    """

    def handleEvent(self,event):
        if event.type == QUIT:
            print("Attempted Quit")
            self.alive = False
        if event.type == KEYDOWN:
            if self.getKey(K_ESCAPE):
                print("Escape key pressed; Aborting.")
                self.alive = False
            elif self.getKey(K_TAB):
                if(self.activeController != -1):
                    blimpIDs = []
                    for connection in self.blimpHandler.connections:
                        if(self.activeController == connection.inputIndex):
                            blimpIDs.append(self.blimpHandler.blimps[connection.blimpIndex].ID)
                    self.blimpHandler.pushMPB(blimpIDs)
            elif self.getKey(K_r):
                if (self.activeController != -1):
                    blimpIDs = []
                    for connection in self.blimpHandler.connections:
                        if (self.activeController == connection.inputIndex):
                            blimpIDs.append(self.blimpHandler.blimps[connection.blimpIndex].ID)
                    self.blimpHandler.requestRecording(blimpIDs)
            elif self.getKey(K_p):
                for blimp in self.blimpHandler.blimps:
                    blimp.auto = 1
            elif self.getNumber()[0]:
                number = self.getNumber()[1]
                if(self.activeController != -1):
                    #remove previous connections
                    if(self.exclusiveConnections):
                        for i in range(0,len(self.blimpHandler.connections)):
                            connection = self.blimpHandler.connections[i]
                            if(connection.inputIndex == self.activeController and connection.blimpIndex != number-1):
                                self.blimpHandler.updateConnection(self.activeController,connection.blimpIndex)
                                i -= 1
                    #add new connection
                    self.blimpHandler.updateConnection(self.activeController,number-1)
                    self.blimpHandler.fixConnections()

        if event.type == MOUSEBUTTONDOWN:
            if(event.button == 1): #Left Click
                pos = pygame.mouse.get_pos()
                for button in self.buttons:
                    if(self.posInRange(pos,button[0],button[1])):
                        self.handleButton(button[2])
                inputs = self.blimpHandler.inputs
                for i in range(0, len(inputs)):
                    if(self.inRangeInput(pos,i)):
                        self.drawing = True
                        self.drawingIndex = i
        if event.type == MOUSEBUTTONUP:
            if(event.button == 1): #Left Click
                pos = pygame.mouse.get_pos()
                inputs = self.blimpHandler.inputs
                blimps = self.blimpHandler.blimps

                for i in range(0,len(inputs)):
                    if(self.inRangeInput(pos,i)):
                        if(self.activeController == i):
                            self.activeController = -1
                        else:
                            self.activeController = i

                for i in range(0,len(blimps)):
                    if(self.inRangeBlimp(pos,i)):
                        if(self.drawing):
                            self.blimpHandler.updateConnection(self.drawingIndex,i)
                self.drawing = False

    def handleButton(self,name):
        #print("handle button")
        if(name=="InputLabel"):
            print("Did not reinitialize inputs")
            #self.blimpHandler.initInputs()
        elif(name[0:3]=="MPB"):
            print("MPB pressed")
            blimpID = int(name[3:])
            self.blimpHandler.pushMPB(blimpID)


    def posInRange(self,pos,origin,size):
        xrange = (origin[0], origin[0] + size[0])
        yrange = (origin[1], origin[1] + size[1])
        validX = xrange[0] <= pos[0] and pos[0] <= xrange[1]
        validY = yrange[0] <= pos[1] and pos[1] <= yrange[1]
        return validX and validY

    def inRangeInput(self, pos, index):
        size = self.blimpHandler.inputs[index].getNameSurface().get_size()
        inputTextY = self.anchor_y_inputText + index*self.spacing_y_inputText
        return self.posInRange(pos,(self.align_input_right - size[0],inputTextY),size)

    def inRangeBlimp(self, pos, index):
        size = self.blimpHandler.blimps[index].getNameSurface().get_size()
        blimpTextY = self.anchor_y_blimpText + index*self.spacing_y_blimpText
        return self.posInRange(pos,(self.align_blimps_left,blimpTextY),size)

    """
    def update(self):
        for event in pygame.event.get():
            self.handleEvent(event)
        self.draw()
    """

    def updateEvent(self):
        for event in pygame.event.get():
            self.handleEvent(event)

    def updateDraw(self):
        self.draw()

    def draw(self):
        self.drawConnections()
        #self.drawActiveController()
        self.drawMisc()
        pygame.display.update()

    def drawConnections(self):
        inputs = self.blimpHandler.inputs
        blimps = self.blimpHandler.blimps

        #backWidth = self.width_inputVisual + self.width_input + self.width_blimps + self.width_blimpState + self.width_blimpStatus
        backWidth = self.width_screen
        pygame.draw.rect(self.screen,Color(150,150,150),Rect(self.anchor_x_inputVisual,self.anchor_y_inputVisual,backWidth,self.height_screen)) #Draw background
        self.screen.blit(self.getTextSurface("Input:",50),(self.anchor_x_input,self.anchor_y_input))
        self.screen.blit(self.getTextSurface("Blimps:",50),(self.anchor_x_blimps,self.anchor_y_blimps))
        self.screen.blit(self.getTextSurface("State",30),(self.anchor_x_blimpState,self.anchor_y_blimpState))
        self.screen.blit(self.getTextSurface("Status",30),(self.anchor_x_blimpStatus,self.anchor_y_blimpStatus))
        #Iterate through inputs
        for i in range(0,len(inputs)):
            #Render inputs
            inputSurface = inputs[i].getNameSurface()
            inputTextX = self.align_input_right-inputSurface.get_size()[0]
            inputTextY = self.anchor_y_inputText + i*self.spacing_y_inputText
            self.screen.blit(inputSurface,(inputTextX,inputTextY))
            #Render input visuals
            rectX = self.anchor_x_inputVisualRect
            rectY = self.anchor_y_inputVisualRect + i*self.spacing_y_inputText
            pygame.draw.rect(self.screen, self.color_inputVisual_background, Rect(rectX, rectY, self.width_inputVisualRect, self.height_inputVisualRect))
            pygame.draw.line(self.screen, self.color_inputVisual_grid, (rectX, rectY + 0.5*self.height_inputVisualRect), (rectX+self.width_inputVisualRect, rectY + 0.5*self.height_inputVisualRect))
            pygame.draw.line(self.screen, self.color_inputVisual_grid, (rectX+0.25*self.width_inputVisualRect, rectY), (rectX+0.25*self.width_inputVisualRect, rectY+self.height_inputVisualRect))
            pygame.draw.line(self.screen, self.color_inputVisual_grid, (rectX+0.75*self.width_inputVisualRect, rectY), (rectX+0.75*self.width_inputVisualRect, rectY+self.height_inputVisualRect))
            input = self.blimpHandler.inputs[i].grabInput()
            leftJS = (rectX+0.25*self.width_inputVisualRect + input[0] * self.inputVisualRect_radius, rectY + 0.5*self.height_inputVisualRect - input[1] * self.inputVisualRect_radius)
            rightJS = (rectX+0.75*self.width_inputVisualRect + input[2] * self.inputVisualRect_radius, rectY + 0.5*self.height_inputVisualRect - input[3] * self.inputVisualRect_radius)
            pygame.draw.circle(self.screen, self.color_inputVisual_joystick, leftJS, self.inputVisualRect_JSCircle)
            pygame.draw.circle(self.screen, self.color_inputVisual_joystick, rightJS, self.inputVisualRect_JSCircle)


        #Render ActiveController Indicator
        if(self.activeController != -1):
            indicatorX = self.anchor_x_activeControllerIndicator
            indicatorY = self.anchor_y_activeControllerIndicator + self.activeController * self.spacing_y_inputText
            pygame.draw.circle(self.screen, self.activeColor, (indicatorX, indicatorY), 5)

        #Render list of blimps
        for i in range(0,len(blimps)):
            if(blimps[i].auto == 1):
                blimpColor = self.color_blimpState_autonomous
            else:
                blimpColor = self.color_blimpState_manual
            blimpSurface = self.getTextSurface(blimps[i].name, int(40 - len(blimps[i].name)),blimpColor)
            blimpSurface = blimpSurface.convert_alpha()
            #Render blimp heartbeats
            blimp = blimps[i]
            heartbeatWidth = blimpSurface.get_width()*blimp.lastHeartbeatDiff/blimp.heartbeatDisconnectDelay
            heartbeatRect = Rect(0,0,heartbeatWidth,blimpSurface.get_height())
            blimpSurface.fill(Color(255,255,255,50),heartbeatRect,special_flags=BLEND_RGBA_ADD)
            blimpTextX = self.align_blimps_left
            blimpTextY = self.anchor_y_blimpText + i*self.spacing_y_blimpText
            self.screen.blit(blimpSurface,(blimpTextX,blimpTextY))
            self.screen.blit(self.stateSurfaceMap[blimp.receivedState],(self.anchor_x_blimpState,blimpTextY))

        #Render connection line if mouse is to the right of inputs
        newLineColor = Color(255,100,255)
        lineColor = Color(255,255,255)
        lineThickness = 3
        pos = pygame.mouse.get_pos()
        if(self.drawing and pos[0]>self.align_input_right):
            inputSurfaceSize = inputs[self.drawingIndex].getNameSurface().get_size()
            inputPos = (self.align_input_right,int(self.anchor_y_inputText + self.drawingIndex*self.spacing_y_inputText+inputSurfaceSize[1]/2))
            pygame.draw.line(self.screen,newLineColor,inputPos,pos,lineThickness)

        #Render connection lines
        for connection in self.blimpHandler.connections:
            inputIndex = connection.inputIndex
            blimpIndex = connection.blimpIndex
            inputSurfaceSize = inputs[inputIndex].getNameSurface().get_size()
            blimpSurfaceSize = blimps[blimpIndex].getNameSurface().get_size()
            inputPos = (self.align_input_right,int(self.anchor_y_inputText + inputIndex*self.spacing_y_inputText+inputSurfaceSize[1]/2))
            blimpPos = (self.align_blimps_left,int(self.anchor_y_blimpText + blimpIndex*self.spacing_y_blimpText+blimpSurfaceSize[1]/2))
            pygame.draw.line(self.screen,lineColor,inputPos,blimpPos,lineThickness)

    def drawActiveController(self):
        activeColor = Color(0,255,255)
        whiteColor = Color(255,255,255)

        #Joystick variables
        JSLeftOrigin = (600+200,200)
        JSRightOrigin = (1000+200,200)
        circleRadius = 25
        joystickRadius = 200-circleRadius

        #Clamp variables
        CLeftOrigin = (490+200,520)
        CRightOrigin = (510+200,520)
        CLength = 70

        #Draw background with active outline
        pygame.draw.rect(self.screen,activeColor,Rect(400+200,0,800,600))
        pygame.draw.rect(self.screen,Color(100,100,100),Rect(405+200,5,790,390))
        pygame.draw.rect(self.screen,Color(100,100,100),Rect(405+200,405,790,190))

        #Display Clamp text
        self.screen.blit(self.textSurface_Clamp, (465+200,542))

        if(self.activeController==-1):
            #Joysticks
            pygame.draw.circle(self.screen,whiteColor,JSLeftOrigin,circleRadius)
            pygame.draw.circle(self.screen,whiteColor,JSRightOrigin,circleRadius)
            #Clamp
            pygame.draw.line(self.screen,whiteColor,CLeftOrigin,self.getClampPoint(CLeftOrigin, CLength,180),5)
            pygame.draw.line(self.screen,whiteColor,CRightOrigin,self.getClampPoint(CRightOrigin, CLength,0),5)
        else:
            #Joysticks
            input = self.blimpHandler.inputs[self.activeController].getInput()
            leftPos = (JSLeftOrigin[0]+joystickRadius*input[0],JSLeftOrigin[1]-joystickRadius*input[1])
            rightPos = (JSRightOrigin[0]+joystickRadius*input[2],JSRightOrigin[1]-joystickRadius*input[3])
            pygame.draw.circle(self.screen,whiteColor,leftPos,circleRadius)
            pygame.draw.circle(self.screen,whiteColor,rightPos,circleRadius)
            #Clamp
            clampValue = 0
            pygame.draw.line(self.screen, whiteColor, CLeftOrigin, self.getClampPoint(CLeftOrigin, CLength, 180-clampValue*90), 5)
            pygame.draw.line(self.screen, whiteColor, CRightOrigin, self.getClampPoint(CRightOrigin, CLength, clampValue*90), 5)

    def getTextSurface(self, text, size, color=None):
        if(color == None):
            color = Color(0,0,0)
        textKey = text + "," + str(size) + "," + str(color)
        surface = self.textSurfaces.get(textKey)
        if(surface == None):
            surface = getTextSurface(text,size,color)
            self.textSurfaces[textKey] = surface
            #print("New key:",textKey)
            #print(len(self.textSurfaces.keys()))
        return surface

    def drawMisc(self):
        #self.screen.blit(self.textSurface_Record, (self.anchor_x_keybind,self.anchor_y_keybind))
        #self.screen.blit(self.textSurface_Panic, (self.anchor_x_keybind,self.anchor_y_keybind+self.spacing_y_keybind))
        self.screen.blit(self.getTextSurface("Manual",30,self.color_blimpState_manual),(self.anchor_x_blimpStateLegend,self.anchor_y_blimpStateLegend))
        self.screen.blit(self.getTextSurface("Autonomous",30,self.color_blimpState_autonomous),(self.anchor_x_blimpStateLegend,self.anchor_y_blimpStateLegend+self.spacing_y_blimpStateLegend))
        stringBarometer = "Barometer: "
        if(self.blimpHandler.baroType == None):
            stringBarometer += "Disconnected"
        else:
            stringBarometer += "(" + self.blimpHandler.baroType + ") " + str(self.blimpHandler.baseHeight)
        self.screen.blit(self.getTextSurface(stringBarometer, 30),(self.anchor_x_barometer, self.anchor_y_barometer))

    def getClampPoint(self, startingPoint, length, angleDegrees):
        angle = angleDegrees/180*math.pi
        return (startingPoint[0]+length*math.cos(angle), startingPoint[1]-length*math.sin(angle))

    def removeBlimp(self, blimpID):
        for i in range(0,len(self.buttons)):
            if(self.buttons[i][2]=="MPB"+str(blimpID)):
                self.buttons.pop(i)

    def getElementY(self, index):
        return 60+60*index

    def getKey(self, key):
        return pygame.key.get_pressed()[key]

    def getNumber(self):
        numIDs = [K_1, K_2, K_3, K_4, K_5, K_6, K_7, K_8, K_9, K_0]
        for i in range(0,len(numIDs)):
            if(self.getKey(numIDs[i])): return (True,i+1)
        return (False,-1)

    def close(self):
        print("Closing Display")
        pygame.quit()
        sys.exit()

    def buttonLabelExists(self, label):
        for button in self.buttons:
            if(button[2] == label):
                return True
        return False