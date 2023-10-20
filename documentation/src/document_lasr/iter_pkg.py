import os
from pathlib import PurePath

FWD = os.path.dirname(os.path.realpath(__file__))

# Scan upwards to find the workspace root
def is_root(path):
    return os.path.exists(os.path.join(path, 'build')) \
        and os.path.exists(os.path.join(path, 'devel'))

WORKSPACE_ROOT = FWD
while not is_root(WORKSPACE_ROOT) \
    and os.path.dirname(WORKSPACE_ROOT) != WORKSPACE_ROOT:
    WORKSPACE_ROOT = os.path.abspath(os.path.join(WORKSPACE_ROOT, ".."))

if not is_root(WORKSPACE_ROOT):
    raise Exception("Could not find workspace root, have you run `catkin build` yet?")

SRC_DIR = os.path.join(WORKSPACE_ROOT, 'src')

print('Workspace root:', WORKSPACE_ROOT)

# List out all packages in workspace
def pkg_list():
    packages = []
    for root, dirs, files in os.walk(SRC_DIR):
        for file in files:
            if file == 'package.xml':
                fullname = os.path.join(root, file)
                packages.append(root)
    
    return packages

# Check if this is a LASR package
def is_lasr_pkg(path):
    relative_path = path[len(SRC_DIR):].lower()
    pure_path = PurePath(relative_path)
    parts = list(pure_path.parts)

    # Remove root component
    if parts[0] == os.sep:
        parts.pop(0)
    
    # Guess if this is part of LASR
    return parts[0] in ['base', 'lasr-base', 'lasr_base']

# List out all LASR packages
def pkg_lasr_list():
    return [pkg for pkg in pkg_list() if is_lasr_pkg(pkg)]
