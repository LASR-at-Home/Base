#!/usr/bin/env python3
import document_lasr
import subprocess
import shutil
import os

# Determine dest folder
FWD = os.path.dirname(os.path.realpath(__file__))
WEB_DIR = os.path.abspath(os.path.join(FWD, '..', 'web'))
DOC_DIR = os.path.join(WEB_DIR, 'docs', 'packages')

# Clear files
if os.path.exists(DOC_DIR):
    shutil.rmtree(DOC_DIR)

# Make the directory
os.mkdir(DOC_DIR)

# Copy README files
for pkg in document_lasr.pkg_lasr_list():
    README = os.path.join(pkg, 'README.md')
    if os.path.exists(README):
        shutil.copyfile(README, os.path.join(DOC_DIR, os.path.basename(pkg) + '.md'))

# Change directory
os.chdir(WEB_DIR)

# Install packages
subprocess.call("npm i", shell=True)
subprocess.call("npm run build", shell=True)
subprocess.call("npm run serve", shell=True)
