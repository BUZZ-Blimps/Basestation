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
    s.bind(('192.168.0.200', 12345))
    print("Bound socket.")
    

    dat = b''
    #dump_buffer(s)
    
  # image = np.asarray(bytearray(resp.read()), dtype="uint8")
      
    # use imdecode function
    # image = cv2.imdecode(image, cv2.IMREAD_COLOR)

    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        if struct.unpack("B", seg[0:1])[0] > 1:
            dat += seg[1:]
        else:
            dat += seg[1:]
            img = cv2.imdecode(np.asarray(bytearray(dat), dtype='uint8'), cv2.IMREAD_COLOR)

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
