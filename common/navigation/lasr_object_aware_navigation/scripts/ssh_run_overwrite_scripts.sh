#!/bin/bash

# ========================
# Configuration
# ========================

REMOTE_USER="pal"
REMOTE_HOST="tiago-157c"
# todo: NEVER PUT REAL PASSWORD HERE!
SSH_PASSWORD="your_ssh_password_here"
SUDO_PASSWORD="your_sudo_password_here"
SSH_PASSWORD="pal"
SUDO_PASSWORD="palroot"

LOCAL_SCRIPT="overwrite_scripts.sh"
REMOTE_SCRIPT="/home/${REMOTE_USER}/overwrite_scripts.sh"

# ============================================
# Check dependencies
# ============================================

if ! command -v sshpass >/dev/null 2>&1; then
  echo "[ERROR] Please install sshpass first: sudo apt install sshpass"
  exit 1
fi

if [ ! -f "$LOCAL_SCRIPT" ]; then
  echo "[ERROR] Local script '$LOCAL_SCRIPT' not found."
  exit 1
fi

# ============================================
# Copy script to the remote robot
# ============================================

echo "[INFO] Uploading script to robot..."
sshpass -p "$SSH_PASSWORD" scp "$LOCAL_SCRIPT" ${REMOTE_USER}@${REMOTE_HOST}:"$REMOTE_SCRIPT"

# ============================================
# Run script remotely with sudo and cleanup
# ============================================

echo "[INFO] Executing script remotely with sudo..."
sshpass -p "$SSH_PASSWORD" ssh -tt ${REMOTE_USER}@${REMOTE_HOST} << EOF
  echo "$SUDO_PASSWORD" | sudo -S bash -c 'export TMPDIR=/tmp && bash "$REMOTE_SCRIPT"'
  sudo rm -f "$REMOTE_SCRIPT"
  exit
EOF

echo "[DONE] Script executed and removed remotely."
