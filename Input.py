import pygame
from pygame.locals import *
from Text import getTextSurface
import time

class Input:
    def __init__(self, type, name, data):
        self.type = type
        self.name = name
        self.nameSurface = getTextSurface(self.name,30)
        if(type=="Keyboard"):
            self.keys = data #Key constants
        elif(type=="Controller"):
            self.controller = data #Controller object

        self.recordedInput = ()

        self.keyboardActionMapping = {"grab":K_g,
                                      "auto":K_a,
                                      "shoot":K_l,
                                      "panicAuto":K_p,
                                      "kill":K_k,
                                      "record":K_r,
                                      "connectDown":None,
                                      "connectUp":None}

        self.controllerActionMapping = {"grab":"BUMPER_RIGHT",
                                        "auto":"TRIGGER_RIGHT",
                                        "shoot":"BUMPER_LEFT",
                                        "panicAuto":"Y",
                                        "kill":"X",
                                        "record":"A",
                                        "connectDown":"DPAD_DOWN",
                                        "connectUp":"DPAD_UP"}

        self.actionElapsedTimeRequired = {"grab":0,
                                          "auto":0,
                                          "shoot":0,
                                          "panicAuto":0.5,
                                          "kill":1,
                                          "record":0,
                                          "connectDown":0,
                                          "connectUp":0}

        self.controllerInputMapping = {"Xbox 360 Wireless Receiver":  #in use
                                      {"JS_LEFT_X":"A0",
                                       "JS_LEFT_Y":"A1",
                                       "TRIGGER_LEFT":"A2",
                                       "JS_RIGHT_X":"A3",
                                       "JS_RIGHT_Y":"A4",
                                       "TRIGGER_RIGHT":"A5",
                                       "A":"B0",
                                       "B":"B1",
                                       "X":"B2",
                                       "Y":"B3",
                                       "BUMPER_LEFT":"B4",
                                       "BUMPER_RIGHT":"B5",
                                       "BACK":"B6",
                                       "START":"B7",
                                       "HOME":"B8",
                                       "JS_LEFT_BUTTON":"B9",
                                       "JS_RIGHT_BUTTON":"B10",
                                       "DPAD_LEFT":"B11",
                                       "DPAD_RIGHT":"B12",
                                       "DPAD_UP":"B13",
                                       "DPAD_DOWN":"B14"},
                                  "Xbox Series X Controller":
                                      {"JS_LEFT_X":"A0",
                                       "JS_LEFT_Y":"A1",
                                       "TRIGGER_LEFT":"A2",
                                       "JS_RIGHT_X":"A3",
                                       "JS_RIGHT_Y":"A4",
                                       "TRIGGER_RIGHT":"A5",
                                       "A":"B0",
                                       "B":"B1",
                                       "X":"B2",
                                       "Y":"B3",
                                       "BUMPER_LEFT":"B4",
                                       "BUMPER_RIGHT":"B5",
                                       "BACK":"B10",
                                       "START":"B11",
                                       "JS_LEFT_BUTTON":"B13",
                                       "JS_RIGHT_BUTTON":"B14",
                                       "DPAD_LEFT":"H00-",
                                       "DPAD_RIGHT":"H00+",
                                       "DPAD_UP":"H01+",
                                       "DPAD_DOWN":"H01-"},
                                  "Xbox 360 Controller":
                                      {"JS_LEFT_X": "A0",
                                       "JS_LEFT_Y": "A1",
                                       "TRIGGER_LEFT": "A2",
                                       "JS_RIGHT_X": "A3",
                                       "JS_RIGHT_Y": "A4",
                                       "TRIGGER_RIGHT": "A5",
                                       "A": "B0",
                                       "B": "B1",
                                       "X": "B2",
                                       "Y": "B3",
                                       "BUMPER_LEFT": "B4",
                                       "BUMPER_RIGHT": "B5",
                                       "BACK": "B6",
                                       "START": "B7",
                                       "JS_LEFT_BUTTON": "B9",
                                       "JS_RIGHT_BUTTON": "B10",
                                       "DPAD_LEFT": "H00-",
                                       "DPAD_RIGHT": "H00+",
                                       "DPAD_UP": "H01+",
                                       "DPAD_DOWN": "H01-"}
                                  }

        self.actionStartTime = {}
        self.actionStates = {}
        self.actionObserved = {}

        if(self.type == "Keyboard"):
            for key in self.keyboardActionMapping.keys():
                self.actionStates[key] = False
                self.actionStartTime[key] = -1
                self.actionObserved[key] = False
        elif(self.type == "Controller"):
            for key in self.controllerActionMapping.keys():
                self.actionStates[key] = False
                self.actionStartTime[key] = -1
                self.actionObserved[key] = False

        self.prevPressGrab = 0
        self.currentPressGrab = 0
        self.prevPressAuto = 0
        self.currentPressAuto = 0
        self.currentPressConnect = 0
        self.prevPressShoot = 0
        self.currentPressShoot = 0

        self.pressingPanicAuto = False
        self.pressPanicAutoStartTime = 0
        self.panicTriggerTime = 0.5

        self.pressingKill = False
        self.pressKillStartTime = 0
        self.killTriggerTime = 1

        self.vibrateUntilTime = 0

    def update(self):
        #Inputs
        self.recordedInput = self.__getInput()
        #Actions
        self.__updateActions()

    def __updateActions(self):
        currentTime = time.time()
        if(self.type == "Keyboard"):
            for actionName in self.keyboardActionMapping.keys():
                if(self.keyboardActionMapping[actionName] == None):
                    continue
                inputState = self.getKey(self.keyboardActionMapping[actionName])
                startTime = self.actionStartTime[actionName]
                elapsedTimeRequired = self.actionElapsedTimeRequired[actionName]
                enoughTime = currentTime - startTime >= elapsedTimeRequired
                observed = self.actionObserved[actionName]
                if(not inputState and startTime == -1):
                    pass
                elif(not inputState and startTime != -1):
                    self.actionStates[actionName] = False
                    self.actionStartTime[actionName] = -1
                elif(inputState and startTime != -1 and not enoughTime):
                    pass
                elif(inputState and startTime != -1 and enoughTime and not observed):
                    self.actionStates[actionName] = True
                    self.actionObserved[actionName] = True
                elif(inputState and startTime != -1 and enoughTime and observed):
                    self.actionStates[actionName] = False
                elif(inputState and startTime == -1):
                    self.actionStartTime[actionName] = currentTime
                    self.actionObserved[actionName] = False
        elif(self.type == "Controller"):
            for actionName in self.controllerActionMapping.keys():
                if(self.controllerActionMapping[actionName] == None):
                    continue
                inputState = 1 if self.getControllerInput(self.controllerActionMapping[actionName]) > 0.5 else 0
                startTime = self.actionStartTime[actionName]
                elapsedTimeRequired = self.actionElapsedTimeRequired[actionName]
                enoughTime = currentTime - startTime >= elapsedTimeRequired
                observed = self.actionObserved[actionName]
                if (not inputState and startTime == -1):
                    pass
                elif (not inputState and startTime != -1):
                    self.actionStates[actionName] = False
                    self.actionStartTime[actionName] = -1
                elif (inputState and startTime != -1 and not enoughTime):
                    pass
                elif (inputState and startTime != -1 and enoughTime and not observed):
                    self.actionStates[actionName] = True
                    self.actionObserved[actionName] = True
                elif (inputState and startTime != -1 and enoughTime and observed):
                    self.actionStates[actionName] = False
                elif (inputState and startTime == -1):
                    self.actionStartTime[actionName] = currentTime
                    self.actionObserved[actionName] = False

    def grabInput(self):
        return self.recordedInput

    def grabAction(self,actionName):
        return self.actionStates[actionName]

    def getNameSurface(self):
        return self.nameSurface

    def __getInput(self):
        if(self.type == "Keyboard"):
            return self.getInputKeyboard()
        elif(self.type == "Controller"):
            return self.getInputController()

    def getInputKeyboard(self):
        # Input
        keys = self.keys #KeyConstants=[right,left,forward,backward,up,down,morePower,grab,auto]
        powerNormal = 0.3
        powerAdd = 0.2
        power = powerNormal + powerAdd * self.getKey(keys[6])

        leftX = self.getKey(keys[0]) - self.getKey(keys[1])
        leftY = self.getKey(keys[2]) - self.getKey(keys[3])
        rightX = 0 #self.getKey(keys[3]) - self.getKey(keys[4])
        rightY = self.getKey(keys[4]) - self.getKey(keys[5])

        leftX *= (power+0.3)
        leftY *= (power+0.3)
        rightX *= power
        rightY *= (power+0.5)

        # Enforce deadzones
        leftX = fixInput(leftX)
        rightX = fixInput(rightX)
        leftY = fixInput(leftY)
        rightY = fixInput(rightY)

        # Actions
        """
        currentTime = time.time()
        for key in self.keyboardActionMapping.keys():
            inputState = self.getKey(self.keyboardActionMapping[key])
            if(inputState and self.actionStates[key] == False):
                self.actionStates[key] = True
                self.actionStartTime[key] = currentTime
                self.actionObserved[key] = False
            elif(not inputState and self.actionStates[key] == True):
                self.actionStates[key] = False
                self.actionStartTime = -1
                self.actionObserved[key] = False
        """

        #Other Input
        self.currentPressGrab = self.getKey(keys[7])
        self.currentPressAuto = self.getKey(keys[8])

        return [leftX, leftY, rightX, rightY]

    def getInputController(self):
        # Input
        controller = self.controller
        leftX = self.getControllerInput("JS_LEFT_X")
        leftY = -1 * self.getControllerInput("JS_LEFT_Y")
        #rightX = controller.get_axis(2) 2=left js; 5=right trigger
        rightX = self.getControllerInput("JS_RIGHT_X")
        rightY = -1 * self.getControllerInput("JS_RIGHT_Y")

        # Enforce deadzones
        leftX = fixInput(leftX)
        rightX = fixInput(rightX)
        leftY = fixInput(leftY)
        rightY = fixInput(rightY)

        # Actions
        """
        currentTime = time.time()
        for key in self.controllerActionMapping.keys():
            inputState = 1 if self.getControllerInput(self.controllerActionMapping[key]) > 0.5 else 0
            if (inputState and self.actionStates[key] == False):
                self.actionStates[key] = True
                self.actionStartTime[key] = currentTime
                self.actionObserved[key] = False
            elif (not inputState and self.actionStates[key] == True):
                self.actionStates[key] = False
                self.actionStartTime = -1
                self.actionObserved[key] = False
        """

        self.currentPressGrab = 1 if self.getControllerInput("BUMPER_RIGHT") > 0.5 else 0
        self.currentPressAuto = 1 if self.getControllerInput("TRIGGER_RIGHT") > 0.5 else 0
        self.currentPressConnect = 1 if self.getControllerInput("DPAD_DOWN") > 0.5 else 0
        self.currentPressShoot = 1 if self.getControllerInput("BUMPER_LEFT") > 0.5 else 0
        #right/left, forward/backward, 0, up/down
        """
        if(controller.get_name() == "Xbox Series X Controller"):
            keys = self.controllerMapping["Xbox Series X Controller"].keys()
            for key in keys:
                print(key,": ",self.getControllerInput(key),sep="",end="  ")
            print()
        """

        # print("0: ", self.controller.get_button(0))
        # print("1: ", self.controller.get_button(1))
        # print("2: ", self.controller.get_button(2))
        # print("3: ", self.controller.get_button(3))
        # print("4 ", self.controller.get_button(4))
        # print("5 ", self.controller.get_button(5))
        # print("6 ", self.controller.get_button(6))
        # print("7 ", self.controller.get_button(7))
        # print("8 ", self.controller.get_button(8))

        #Panic Auto
        if(self.getControllerInput("Y") == 1):
            if(not self.pressingPanicAuto):
                self.pressingPanicAuto = True
                self.pressPanicAutoStartTime = time.time()
        elif(self.pressingPanicAuto):
            self.pressingPanicAuto = False

        #Kill
        if(self.getControllerInput("X") == 1):
            if not self.pressingKill:
                self.pressingKill = True
                self.pressKillStartTime = time.time()
        elif self.pressingKill:
            self.pressingKill = False


        #Kill program

        #Vibration
        if(self.vibrateUntilTime > time.time()):
            controller.rumble(1, 1, 0)
        else:
            controller.stop_rumble()

        #controller.rumble(10,20,2)

        mode = 1
        if(mode == 1):
            return [leftX, leftY, rightX, rightY]
        elif(mode == 2):
            return [leftX, rightY, rightX, leftY]

    def getKey(self, key):
        return pygame.key.get_pressed()[key]

    def getControllerInput(self,inputName):
        controllerName = self.controller.get_name()
        #if(controllerName == "Xbox 360 Controller"):
            #self.dumpControllerData()
        inputSource = self.controllerInputMapping[controllerName][inputName]
        if(inputSource[0] == "A"):
            axisNum = int(inputSource[1:])
            return self.controller.get_axis(axisNum)
        elif(inputSource[0] == "B"):
            buttonNum = int(inputSource[1:])
            return self.controller.get_button(buttonNum)
        elif(inputSource[0] == "H"):
            hatNum = int(inputSource[1])
            hatIndex = int(inputSource[2])
            retCondition = inputSource[3]
            inputValue = self.controller.get_hat(hatNum)[hatIndex]
            if(retCondition == "+" and inputValue == 1):
                return 1
            elif(retCondition == "-" and inputValue == -1):
                return 1
        return 0

    def notify(self, timeDuration):
        if(self.type == "Controller"):
            #print("Notify")
            self.vibrateUntilTime = time.time() + timeDuration

    def trigger_panicAuto(self):
        return self.pressingPanicAuto and time.time() - self.pressPanicAutoStartTime > self.panicTriggerTime

    def trigger_kill(self):
        return self.pressingKill and time.time() - self.pressKillStartTime > self.killTriggerTime

    def trigger_connectToBlimp(self):
        return self.currentPressConnect

    def dumpControllerData(self):
        for i in range(8): #Axes
            try:
                print("Axis ",i,": ",self.controller.get_axis(i),sep='')
            except:
                pass
        for i in range(15): #Buttons
            try:
                print("Button ",i,": ",self.controller.get_button(i),sep='')
            except:
                pass
        for i in range(3): #Hats
            try:
                print("Hat ",i,": ",self.controller.get_hat(i),sep='')
            except:
                pass
        print("\n");
        time.sleep(0.5)

def fixInput(x, deadZero=0.1, deadOne=0.01, decimals=2):
    if (abs(x) < deadZero): return 0
    if (x > 1 - deadOne): return 1
    if (x < -1 + deadOne): return -1
    return round(x, decimals)
