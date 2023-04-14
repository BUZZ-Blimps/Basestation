#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import socket
import struct
import math

class FrameSegment(object):
    """ 
    Object to break down image frame segment
    if the size of image exceed maximum datagram size 
    """
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64 # extract 64 bytes in case UDP frame overflown
    def __init__(self, sock, port, addr="127.0.0.1"):
        self.s = sock
        self.port = port
        self.addr = addr

    def udp_frame(self, img):
        """ 
        Compress image and Break down
        into data segments 
        """
        compress_img = cv2.imencode('.jpg', img)[1]
        dat = compress_img.tostring()
        print("Dat type:",type(dat))
        size = len(dat)
        count = math.ceil(size/(self.MAX_IMAGE_DGRAM))
        array_pos_start = 0
        while count:
            array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
            self.s.sendto(struct.pack("B", count) +
                dat[array_pos_start:array_pos_end], 
                (self.addr, self.port)
                )
            array_pos_start = array_pos_end
            count -= 1

def dump_buffer(s):
    """ Emptying buffer frame """
    MAX_DGRAM = 2**16
    while True:
        try:
            seg, addr = s.recvfrom(MAX_DGRAM)
        except socket.timeout:
            print("Socket timed out.")
            continue
        val = struct.unpack("B", seg[0:1])[0]
        print("Val = ", val)
        if val == 1:
            print("finish emptying buffer")
            break

def listening(s,dat):
    MAX_DGRAM = 2**16
    try:
        seg, addr = s.recvfrom(MAX_DGRAM)
    except socket.timeout:
    	pass
    
    if struct.unpack("B", seg[0:1])[0] > 1:
        dat += seg[1:]
    return dat

def main():

    """
    while True:
        _, frame = cap.read()
        cv2.imshow("Frame",frame)
        cv2.waitKey(1);
    """

    """ Top level main function """
    # Set up UDP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1)
    port = 12345

    fs = FrameSegment(s, port)
    print("Frames done")

    """
    dat = b''
    dump_buffer(s)
    print("Dumped Buffer")
    dat = listening(s,dat)
    print("Started Listening")
    """
    
    # Open camera
    print("Opening camera... ",end='')
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print("Done!")

    while (cap.isOpened()):
        _, frame = cap.read()
        fs.udp_frame(frame)
    cap.release()
    cv2.destroyAllWindows()
    s.close()

if __name__ == "__main__":
    main()
