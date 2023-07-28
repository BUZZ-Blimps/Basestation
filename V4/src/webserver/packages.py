# Python Packages
from threading import Thread
import time
import subprocess
import socket
import serial
import signal
import sys
import os
import requests
import fcntl

# Flask Packages
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

# Global View of Files
def globalView():
    import os
    import sys
    # Basestation Node Package
    # Get the directory of the current script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Append the external_folder path relative to the current file's location
    external_folder_path = os.path.join(current_dir, '../')

globalView()

# ROS Packages
from ros.basestation import Basestation

# Database Packages
from database.database import Database

# Flask and SocketIO Packages
from webserver import app, socketio

from globalVars import *
