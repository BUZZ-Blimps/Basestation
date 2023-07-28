#!/bin/bash

# Free Port 5000
./freePort.sh 5000

# Run Main Program
echo "Starting Basestation..."
echo ""
python3 ../src/main.py & python3 ../src/ros/runRos.py
echo ""
echo "Shutting Down Basestation..."