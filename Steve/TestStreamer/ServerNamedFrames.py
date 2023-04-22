#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import socket
import struct
import time

MAX_DGRAM = 2**14

blimpIPNameMap = { "192.168.0.101": "Spicy Hot Dog",
                   "192.168.0.102": "Waffle",
                   "192.168.0.103": "Apple",
                   "192.168.0.104": "Milk",
                   "192.168.0.105": "Pasta",

                   "192,168.0.100": "Silly Ahh",

                   "192.168.0.80": "Big Cup of Eggs",
                   "192.168.0.20": "Leg in a Cup",
                   "192.168.0.89": "I'm in a Cup",
                   "192.168.0.62": "My Cup of Eggs",
                   "192.168.0.86": "Pint of Eggs",
                   "192.168.0.14": "Stealthy Steve",

                   "192.168.0.38": "Barometer"}

def dump_buffer(s):
    """ Emptying buffer frame """
    s.setblocking(False)
    while True:
        try:
            seg, addr = s.recvfrom(MAX_DGRAM)
        except BlockingIOError:
            print("Finished emptying buffer")
            break
    s.setblocking(True)

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
        address = str(addr[0])
        if address in blimpIPNameMap:
            blimpName = blimpIPNameMap[address]
        else:
            blimpName = address

        # print("Received data")
        frameNum = struct.unpack("B", seg[0:1])[0]
        #print(frameNum, ", ",sep='',end='')
        if struct.unpack("B", seg[0:1])[0] > 1:
            dat += seg[1:]
        else:
            #print()
            dat += seg[1:]
            # Get size of frameName (first byte)
            frameNameSize = int(dat[0])
            try:
                frameName = bytes.decode(dat[1:(1+frameNameSize)], "utf-8")
            except:
                print("bytes.decode failed (dat[0:10] = ",dat[0:10],sep='')
                dump_buffer(s)
                dat = b''
                continue

            #print("frameNameSize =",frameNameSize,"\t\tframeName =",frameName)
            dat = dat[(1+frameNameSize):]

            try:
                img = cv2.imdecode(np.fromstring(dat, dtype=np.uint8), 1)

                if (img is not None):
                    cv2.imshow(blimpName + ": "+frameName, img)
            except:
                print("imdecode failed")
                dat = b''
                continue

            # Increment frame count
            if frameName in frameCounts:
                frameCounts[frameName] += 1
            else:
                frameCounts[frameName] = 1

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