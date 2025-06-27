#!/bin/bash

# -------- CONFIGURATION --------
ROBOT_HOST="tiago-157c"               # Your robot's hostname or IP
SSH_USER="pal"                        # SSH username
SSH_PASSWORD="pal"      # SSH password (if using sshpass)
SUDO_PASSWORD="palroot"    # sudo password on the robot
UNIX_NOW=$(date +%s)
ADJUSTED_UTC=$(date -u -d "@$(echo "$UNIX_NOW + 1.5" | bc)" +"%Y-%m-%d %H:%M:%S") # Generate current local time

# -------- CHECK: sshpass is installed --------
if ! command -v sshpass >/dev/null 2>&1; then
  echo "Please install sshpass first: sudo apt install sshpass"
  exit 1
fi

# -------- EXECUTE REMOTE TIME SYNC AND RESTART DEPLOYER --------
# -------- EXECUTE REMOTE TIME SYNC ONLY --------
sshpass -p "$SSH_PASSWORD" ssh -o StrictHostKeyChecking=no ${SSH_USER}@${ROBOT_HOST} \
  "echo '$SUDO_PASSWORD' | sudo -S date -s \"$ADJUSTED_UTC\""

# run pal_restart_deployer within ssh, cause I cannot make it work within this script
