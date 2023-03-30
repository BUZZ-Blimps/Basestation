#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import socket
import struct
import math
import time

def main():
    """
    # Open camera
    print("Opening camera... ",end='')
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print("Done!")
    """

    """
    while True:
        _, frame = cap.read()
        cv2.imshow("Frame",frame)
        cv2.waitKey(1);
    """

    # Set up socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = "127.0.0.1"
    port = 12345
    MAX_DGRAM = 2**16
    print("Opened socket.")

    print("Sending the time.")
    lastSent = 0
    sendDelay = 1
    while True:
        currentTime = time.time()
        if currentTime - lastSent >= sendDelay:
            lastSent = currentTime

            message = "Time=" + str(currentTime)
            messageBytes = bytes(message, 'utf-8')
            print("Sending:",message)
            s.sendto(messageBytes, (addr, port))


    """ Top level main function """
    # Set up UDP socket
    s.settimeout(1)

    fs = FrameSegment(s, port)
    print("Frames done")

    dat = b''
    dump_buffer(s)
    print("Dumped Buffer")
    dat = listening(s,dat)
    print("Started Listening")

    while (cap.isOpened()):
        _, frame = cap.read()
        fs.udp_frame(frame)
    cap.release()
    cv2.destroyAllWindows()
    s.close()

if __name__ == "__main__":
    main()
