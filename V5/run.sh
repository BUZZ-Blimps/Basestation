#!/bin/bash

IP=$(hostname -I | awk '{print $1}')
URL="http://$IP:5000"

echo ""
echo "Basestation Starting..."
echo ""
echo "Running on $URL"
echo ""
echo "Getting Logs..."

./freePort.sh

if [ "$1" == "-o" ]; then
    xdg-open "$URL" > /dev/null 2>&1;
    ros2 launch base_station.launch.py
else
    ros2 launch base_station.launch.py
fi
