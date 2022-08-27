#!/usr/bin/env python3

import rospy
import rospkg
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

import os

TEMPLATE_ROOT = os.path.join(rospkg.RosPack().get_path("lasr_web_server"), "src", "lasr_web_server", "templates")
HOST = '0.0.0.0'
PORT = 4000

app = Flask(__name__, template_folder = TEMPLATE_ROOT)

@app.route('/')
def render_main():
    return render_template("main.html")


if __name__ == "__main__":
    app.run(host=HOST, port=PORT, debug=True)