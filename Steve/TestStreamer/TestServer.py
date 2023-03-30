#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import socket
import struct

MAX_DGRAM = 2**16

def dump_buffer(s):
    """ Emptying buffer frame """
    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        print(seg[0])
        if struct.unpack("B", seg[0:1])[0] == 1:
            print("finish emptying buffer")
            break

def main():
    # Set up socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Opened socket.")
    s.bind(('127.0.0.1', 12345))
    print("Binded socket.")

    print("Listening...")
    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        message = bytes.decode(seg, 'utf-8')
        print("Received:",message)

    s.close()

if __name__ == "__main__":
    main()