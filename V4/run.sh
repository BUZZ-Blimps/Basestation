#!/bin/bash

URL="http://192.168.0.200:5000"

echo ""
echo "Basestation Starting..."
echo ""
echo "Running on $URL"
echo ""
echo "Getting Logs..."

./freePort.sh

# Use -o flag to open website and run the program
if [ "$1" == "-o" ]; then
    xdg-open "$URL" > /dev/null 2>&1;
    python3 main.py
# No flags just runs the programs
else
    python3 main.py
fi


