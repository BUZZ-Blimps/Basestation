#!/bin/bash

# Run Main Program
echo ""
echo "Starting Basestation..."
echo ""
./freePort.sh 5000 # Free Port 5000
python3 ../src/main.py & python3 ../src/ros/runRos.py
echo ""
echo "Shutting Down Basestation..."