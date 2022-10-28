import pygame.font
from pygame.locals import Color

def getTextSurface(text, size=50):
    font = pygame.font.SysFont("Calibri",size)
    textColor = Color(0,0,0)
    antialias = False
    return font.render(text,antialias,textColor)