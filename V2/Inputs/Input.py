from abc import ABC, abstractmethod

import pygame.font
from pygame.locals import Color


class Input(ABC):
    # ============================== SUPER INIT ====================================
    def __init__(self, name):
        self.name = name
        self.nameSurface = getTextSurface(self.name, 30)

        self.recordedInput = [0, 0, 0, 0]

        self.actionElapsedTimeRequired = {"grab": 0,
                                          "auto": 0,
                                          "shoot": 0,
                                          "panicAuto": 0.5,
                                          "kill": 1,
                                          "record": 0,
                                          "connectDown": 0,
                                          "connectUp": 0,
                                          "vibeRight": 0,
                                          "vibeLeft": 0}

        self.actionStartTime = {}
        self.actionStates = {}
        self.actionObserved = {}

    # ============================== UPDATE ========================================
    def update(self):
        # Inputs
        self.__updateInputs()
        # Actions
        self.__updateActions()

    # ============================== ABSTRACT FUNCTIONS ==============================
    @abstractmethod
    def __updateActions(self):
        pass

    @abstractmethod
    def __updateInputs(self):
        pass

    @abstractmethod
    def notify(self, timeDuration):
        pass

    # ============================== ACCESSOR FUNCTIONS =====================================
    def grabInput(self):
        return self.recordedInput

    def grabAction(self, actionName):
        return self.actionStates[actionName]

    def getNameSurface(self):
        return self.nameSurface

def getTextSurface(text, size=50, color=None):
    font = pygame.font.SysFont("Calibri",size)
    if(color==None):
        textColor = Color(0,0,0)
    else:
        textColor = color
    antialias = False
    return font.render(text,antialias,textColor)