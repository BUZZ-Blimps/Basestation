"""
pip3 install pygame
pip3 install pySerial
pip3 install easygui
sudo apt-get install python3-tk (to install tkinter, needed for easygui)
"""
import pygame
from threading import Thread
import time

from BlimpHandler import BlimpHandler
from Display import Display

def main():
    global alive
    alive = True
    pygame.init()

    blimpHandler = BlimpHandler()
    display = Display(blimpHandler)

    #LOOP
    thread_blimpHandler = Thread(target=asyncBlimpHandler,args={blimpHandler})
    thread_display = Thread(target=asyncDisplayDraw,args={display})

    thread_blimpHandler.start()
    thread_display.start()
    while alive:
        time.sleep(0.001)
        display.updateEvent()
        alive = display.alive
        if(alive==False): print("Display killed.")

    thread_blimpHandler.join()
    thread_display.join()
    blimpHandler.close()
    display.close()

def asyncBlimpHandler(blimpHandler):
    while alive:
        time.sleep(0.001)
        blimpHandler.update()

def asyncDisplayDraw(display):
    while alive:
        time.sleep(0.001)
        display.updateDraw()

if __name__ == '__main__':
    main()