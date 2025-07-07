#!/bin/bash

# ========================
# Configuration
# ========================

# Remote user and host
REMOTE_USER="pal"
REMOTE_HOST="tiago-157c"

# Passwords (do NOT leave in production scripts)
# todo: NEVER PUT REAL PASSWORD HERE!
SSH_PASSWORD="your_ssh_password_here"
SUDO_PASSWORD="your_sudo_password_here"

# Local files to upload
LOCAL_LAUNCH_FILE="../launch/rgbd_cloud_laser.launch"
LOCAL_YAML_FILE="../config/rgbd_cloud_laser.yaml"

# Remote temporary path (must be writable by user)
REMOTE_TEMP_PATH="/home/${REMOTE_USER}/temp_rgbd_upload"

# Final target paths (require sudo)
REMOTE_LAUNCH_PATH="/opt/pal/gallium/share/tiago_laser_sensors/launch/"
REMOTE_YAML_PATH="/opt/pal/gallium/share/tiago_laser_sensors/config/"

# ========================
# Check dependencies
# ========================
if ! command -v sshpass >/dev/null 2>&1; then
  echo "[ERROR] Please install sshpass first: sudo apt install sshpass"
  exit 1
fi

# ========================
# Upload files to temporary user-accessible directory
# ========================
echo "[INFO] Creating temporary directory on remote host..."
sshpass -p "$SSH_PASSWORD" ssh -o StrictHostKeyChecking=no ${REMOTE_USER}@${REMOTE_HOST} "mkdir -p ${REMOTE_TEMP_PATH}"

echo "[INFO] Uploading launch file to temp directory..."
sshpass -p "$SSH_PASSWORD" scp "$LOCAL_LAUNCH_FILE" ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_TEMP_PATH}/

echo "[INFO] Uploading YAML file to temp directory..."
sshpass -p "$SSH_PASSWORD" scp "$LOCAL_YAML_FILE" ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_TEMP_PATH}/

# ========================
# Use sudo to move files to final system directory
# ========================
echo "[INFO] Moving files to system directory with sudo..."
sshpass -p "$SSH_PASSWORD" ssh -tt -o StrictHostKeyChecking=no ${REMOTE_USER}@${REMOTE_HOST} << EOF
echo "$SUDO_PASSWORD" | sudo -S mv ${REMOTE_TEMP_PATH}/$(basename "$LOCAL_LAUNCH_FILE") "$REMOTE_LAUNCH_PATH"
echo "$SUDO_PASSWORD" | sudo -S mv ${REMOTE_TEMP_PATH}/$(basename "$LOCAL_YAML_FILE") "$REMOTE_YAML_PATH"
sudo rmdir ${REMOTE_TEMP_PATH}
exit
EOF

echo "[DONE] Files uploaded and moved with sudo successfully."
