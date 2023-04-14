#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import socket
import struct
import math
import time

class Client:
    def __init__(self):
        # Init empty video capture
        self.cap = self.openCamera()

        # Define communication parameters
        self.addr = "127.0.0.1"
        self.port = 12345
        self.MAX_DGRAM = 2**14
        self.packetNumDigits = 2
        self.dataNameNumDigits = 2
        self.dataTypeMap = {"str": b'0',
                             "numpy.ndarray": b'1'}

        # Init socket
        self.s = self.openSocket()

    def __del__(self):
        print("Shutting down socket... ",end='')
        self.s.close()
        print("Done!")

    def openCamera(self):
        print("Opening camera... ",end='')
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 60)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print("Done!")
        return cap

    def openSocket(self):
        print("Opening socket... ",end='')
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Done!")
        return s

    def getFrame(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("VideoCapture Error?")
                return None
            else:
                return frame
        else:
            None

    # data = object (could be a string or an image)
    # message = <number describing data type> + data
    # packet = <packet number> + <small part of message>
    def sendPacket(self, data, dataName):
        dataType = type(data)
        #print(dataType)
        typeStr = None
        dataToSend = None

        # Assemble complete message
        if dataType == str:
            #print("Sending a string!")
            typeStr = "str"
            message = self.dataTypeMap["str"] + data.encode('utf-8')

        elif dataType == np.ndarray:
            #print("Sending a numpy.ndarray")
            typeStr = "numpy.ndarray"
            compressedImg = cv2.imencode('.jpg', data)[1]
            dataNameLen = len(dataName)
            dataNameLenStr = str(dataNameLen)
            dataNameLenStr = "0"*(self.dataNameNumDigits-len(dataNameLenStr)) + dataNameLenStr
            message = self.dataTypeMap["numpy.ndarray"]
            message += dataNameLenStr.encode('utf-8')
            message += dataName.encode('utf-8')
            message += compressedImg.tobytes()

        # Split into packets
        messageSize = len(message)
        minNumberRequiredPackets = math.ceil(messageSize/self.MAX_DGRAM)
        if minNumberRequiredPackets > 10**self.packetNumDigits:
            print("ERROR: Packet size too large to be split and numbered with,",self.packetNumDigits,"digits. Please increase self.packetNumDigits.")
        packets = []
        # Assemble packets in order
        message_pos_start = 0
        while message_pos_start < messageSize:
            packetNum = int(len(packets))
            packetNumStr = str(packetNum)
            packetNumStr = "0"*(self.packetNumDigits-len(packetNumStr)) + packetNumStr

            packet_length = min(messageSize - message_pos_start, self.MAX_DGRAM - self.packetNumDigits)
            packet = packetNumStr.encode('utf-8') + message[message_pos_start : message_pos_start + packet_length]

            packets.append(packet)
            message_pos_start += packet_length

        # send packets in reverse order
        for i in range(0,len(packets)):
            packetIndex = len(packets)-1-i
            packet = packets[packetIndex]
            #print("Packet=",packetIndex,"Len =",len(packet))
            self.s.sendto(packet, (self.addr, self.port))
            #time.sleep(0.005)
        
        if typeStr == "numpy.ndarray":
            #print("Send image:",dataName)
            pass



def main():
    # Init Client object
    client = Client()
    cv2.namedWindow("test")
    while True:
        frame = client.getFrame()
        if type(frame) == type(None):
            continue
        #scaleFactor = 0.5
        #newSize = (int(frame.shape[1]*scaleFactor), int(frame.shape[0]*scaleFactor))
        #resizedFrame = cv2.resize(frame, newSize, interpolation=cv2.INTER_AREA)
        client.sendPacket(frame, "Raw Frame")
        cv2.imshow("Raw",frame)
        print("sent raw. time=",str(time.time()))
        cv2.waitKey(17)
        #time.sleep(0.01)
        #cv2.waitKey(200)
        #client.sendPacket(resizedFrame, "Resized Frame")
        #cv2.waitKey(200)
        #client.sendPacket("Time: " + str(time.time()), "Time")
        #print("Old size:",frame.shape,"; New size:",resizedFrame.shape, "tuple:",newSize)
    return

if __name__ == "__main__":
    main()
