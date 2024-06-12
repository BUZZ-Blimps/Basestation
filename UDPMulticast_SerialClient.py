import serial
from serial.tools import list_ports
import time

#List out available ports
ports = list_ports.comports()
for port in ports:
    print(port[0])

#Try connection with first port
ser = serial.Serial(ports[0][0],115200,timeout=0.2)
i=0
while not ser.is_open:
    print("Attempting to open port.")
    ser.open()
print("Serial port is open.")


send = "|yo|"
ser.write(send.encode('utf8'))
while True:
    readIn = ser.readline().decode('utf8')
    if(readIn!=""): print(readIn,end='')
    time.sleep(0.1)



