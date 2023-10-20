#!/usr/bin/env python3
import document_lasr
import subprocess

document_lasr.configure_web()
subprocess.call("npm i", shell=True)
subprocess.call("npm run start", shell=True)
