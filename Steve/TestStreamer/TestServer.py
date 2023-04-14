#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import socket
import struct
import errno
import sys

class Server:
    def __init__(self):
        # Define communication parameters
        self.addr = "127.0.0.1"
        self.port = 12345
        self.MAX_DGRAM = 2**15
        self.packetNumDigits = 2
        self.dataNameNumDigits = 2
        self.dataTypeMap = {b'0': "str",
                             b'1': "numpy.ndarray"}

        # Init socket
        self.s = self.openSocket()
        self.packetNum = 0
        self.message = b''

    def __del__(self):
        print("Shutting down socket... ",end='')
        self.s.close()
        print("Done!")

    def openSocket(self):
        print("Opening socket... ",end='')
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((self.addr, self.port))
        s.setblocking(False)
        print("Done!")
        return s

    def safeRecvfrom(self):
        seg = None
        addr = None
        try:
            seg, addr = self.s.recvfrom(self.MAX_DGRAM)
        except BlockingIOError:
            pass
        return seg, addr


    def getMessage(self):
        # print("Listening...")
        seg, addr = self.safeRecvfrom()
        if seg == None:
            # No data avilable
            #print("None")
            return None

        packetNumStr = bytes.decode(seg[0:self.packetNumDigits], 'utf-8')
        self.packetNum = int(packetNumStr)
        print("Packet",packetNumStr)
        # Append in reverse order
        self.message = seg[self.packetNumDigits:] + self.message
        
        # Check if this is the last packet
        if self.packetNum == 0:
            parsedMessage = self.parseMessage(self.message)
            self.message = b''
            return parsedMessage
        else:
            return None

    def parseMessage(self, message):
        #print("Parsing...")
        dataTypeByte = message[0:1]
        dataTypeStr = self.dataTypeMap[dataTypeByte]
        #print("Parsed message of type",dataTypeStr)

        data = message[1:]
        parsedMessage = None
        try:
            if dataTypeStr == "str":
                parsedMessage = bytes.decode(data,'utf-8')
            elif dataTypeStr == "numpy.ndarray":
                dataNameLenStr = bytes.decode(data[0:self.dataNameNumDigits], 'utf-8')
                dataNameLen = int(dataNameLenStr)
                dataName = bytes.decode(data[self.dataNameNumDigits:self.dataNameNumDigits+dataNameLen], 'utf-8')
                print("ImgName:",dataName)
                data = data[self.dataNameNumDigits+dataNameLen:]
                parsedImg = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), 1)
                parsedMessage = (parsedImg, dataName)
        except:
            print("PARSE ERROR.")
            return None
        return parsedMessage, dataTypeStr

# waitKey: escape=27, no_key=-1
def main():
    # Init Server object
    server = Server()
    cv2.namedWindow("Default Window")
    while True:
        messageTuple = server.getMessage()
        if messageTuple == None:
            pass
        else:
            message, messageType = messageTuple
            if messageType == "str":
                #print("Received string:",message)
                pass
            elif messageType == "numpy.ndarray":
                image, imageName = message
                #print("Received image:",imageName)
                cv2.imshow(imageName, image)

        key = cv2.waitKey(1)
        if key != -1:
            break

    cv2.destroyAllWindows()
    return

if __name__ == "__main__":
    main()