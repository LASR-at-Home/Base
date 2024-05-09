#!/usr/bin/env python3
import document_lasr
import os, sys

# Determine package name
if len(sys.argv) < 2:
    print("Usage: rosrun documentation generate_readme.py ros_package_name")
    exit()
else:
    package = sys.argv[1]

# Generate all matching packages
for pkg in document_lasr.pkg_list():
    if os.path.basename(pkg) == package:
        document_lasr.generate_readme(pkg)
