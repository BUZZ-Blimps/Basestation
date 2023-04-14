#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import socket
import struct
import time

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

    frameCounts = {}
    lastPrint = 0
    FPSDuration = 1 # second

    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        # print("Received data")
        if struct.unpack("B", seg[0:1])[0] > 1:
            dat += seg[1:]
        else:
            dat += seg[1:]
            # Get size of frameName (first byte)
            frameNameSize = int(dat[0])
            frameName = bytes.decode(dat[1:(1+frameNameSize)], "utf-8")
            #print("frameNameSize =",frameNameSize,"\t\tframeName =",frameName)
            dat = dat[(1+frameNameSize):]

            # Increment frame count
            if frameName in frameCounts:
                frameCounts[frameName] += 1
            else:
                frameCounts[frameName] = 1

            img = cv2.imdecode(np.fromstring(dat, dtype=np.uint8), 1)

            if (img is not None):
                cv2.imshow("Frame: "+frameName, img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            dat = b''

        if time.time() - lastPrint >= FPSDuration:
            lastPrint = time.time()
            print("FPS:    ")
            for frameName in frameCounts:
                frameCount = frameCounts[frameName]
                frameCounts[frameName] = 0
                print(frameName,"(",frameCount,")   ",sep='',end='')
            print()

    # cap.release()
    cv2.destroyAllWindows()
    s.close()

if __name__ == "__main__":
    main()