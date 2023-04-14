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
    """ Getting image udp frame &
    concate before decode and output image """
    
    print("Starting...")
    # Set up socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Opened socket.")
    #s.bind(('127.0.0.1', 12345))
    s.bind(('192.168.0.200', 12345))
    print("Binded socket.")
    

    dat = b''
    #dump_buffer(s)
    

    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        # print("Received data")
        if struct.unpack("B", seg[0:1])[0] > 1:
            dat += seg[1:]
        else:
            dat += seg[1:]
            img = cv2.imdecode(np.fromstring(dat, dtype=np.uint8), 1)

            if (img is not None):
                cv2.imshow('Received Frame', img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            dat = b''

    # cap.release()
    cv2.destroyAllWindows()
    s.close()

if __name__ == "__main__":
    main()