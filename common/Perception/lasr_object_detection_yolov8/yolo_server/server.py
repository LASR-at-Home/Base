#!/usr/bin/env python3
# YOLOv8 IPC processing server

from multiprocessing.connection import Listener

# Import YOLO runtime
# We keep the source in the src/<pkg> directory so we can migrate easily later.

import sys, os

FWD = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(FWD, "../src/lasr_object_detection_yolov8")))

import yolo

# Put ourselves in the model folder
os.chdir(os.path.abspath(os.path.join(FWD, 'models')))

# Determine HOST, PORT, and SECRET_KEY variables
# This script is run outside of the ROS workspace, so we probably don't have ROS_IP set.
# If it is, just use it, otherwise try to make a guess at our LAN IP address.

import socket
from os import environ

if "HOST" in environ:
    HOST = environ.get("HOST")
elif "ROS_IP" in environ:
    HOST = environ.get("ROS_IP")
else:
    try:
        # if we have an internet connection, figure out where we are being routed through
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        HOST = s.getsockname()[0]
        s.close()
    except Exception:
        # otherwise pick the first available interface
        # may return 127.0.0.1
        HOST = socket.gethostbyname(socket.gethostname())

if "PORT" in environ:
    PORT = int(environ.get('PORT'))
else:
    PORT = 42405

if "SECRET_KEY" in environ:
    SECRET_KEY = bytearray(environ.get('SECRET_KEY'), 'ascii')
else:
    SECRET_KEY = b'ros yolov8 package'

print(f'Listening on {HOST}:{PORT}')

# Start the IPC server
listener = Listener((HOST, PORT), authkey=SECRET_KEY)

# Accept connections forever (or at least until interrupt)
while True:
    try:
        conn = listener.accept()
        print('Accepted connection from:', listener.last_accepted)

        # Accept an infinite amount of requests
        while True:
            # Receive parameters
            dataset = conn.recv()
            min_confidence = conn.recv()
            debug = conn.recv()
            encoding = conn.recv()
            width = conn.recv()
            height = conn.recv()
            image_data = conn.recv()
            
            print(f'Received {encoding} image with dim {width}x{height}, using dataset {dataset} and minimum confidence of {min_confidence}, debugging? {debug}.')

            # Call the detection subroutine
            plotted, detected_objects = yolo.detect(dataset, min_confidence, debug, encoding, width, height, image_data)

            # Send the data back
            if debug:
                conn.send(plotted)

            conn.send(detected_objects)
    except KeyboardInterrupt:
        listener.close()
        break

    except EOFError:
        pass
