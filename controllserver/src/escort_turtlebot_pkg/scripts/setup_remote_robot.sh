#!/bin/bash
# setup_remote_robot.sh
# Run this script from the Control PC to configure a remote TurtleBot3 Raspberry Pi.
# Usage: ./setup_remote_robot.sh [ROBOT_IP] [USERNAME]
# Example: ./setup_remote_robot.sh 192.168.1.100 ubuntu

if [ -z "$1" ]; then
    echo "Usage: ./setup_remote_robot.sh [ROBOT_IP] [USERNAME (default: ubuntu)]"
    echo "Example: ./setup_remote_robot.sh 192.168.1.15 ubuntu"
    exit 1
fi

ROBOT_IP=$1
USERNAME=${2:-ubuntu}
LOCAL_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
LOCAL_SCRIPT="$LOCAL_SCRIPT_DIR/setup_robot.sh"
REMOTE_TEMP_SCRIPT="/home/$USERNAME/setup_robot_tmp.sh"

echo "================================================================"
echo " Deploying Setup Script to Remote Robot: $USERNAME@$ROBOT_IP "
echo "================================================================"

# 1. Check if the local setup_robot.sh exists
if [ ! -f "$LOCAL_SCRIPT" ]; then
    echo "Error: Could not find 'setup_robot.sh' in the scripts directory."
    exit 1
fi

# 2. SCP the script to the remote robot
echo "[1/3] Copying setup script to the robot..."
scp "$LOCAL_SCRIPT" "$USERNAME@$ROBOT_IP:$REMOTE_TEMP_SCRIPT"

if [ $? -ne 0 ]; then
    echo "Error: Failed to connect to the robot via SCP. Check the IP and network."
    exit 1
fi

# 3. SSH into the robot and execute the script
echo "[2/3] Executing setup script remotely (You may be asked for the sudo password)..."
ssh -t "$USERNAME@$ROBOT_IP" "chmod +x $REMOTE_TEMP_SCRIPT && $REMOTE_TEMP_SCRIPT"

# 4. Clean up the script from the remote robot
echo "[3/3] Cleaning up temporary files..."
ssh "$USERNAME@$ROBOT_IP" "rm $REMOTE_TEMP_SCRIPT"

echo "================================================================"
echo " Remote Setup Complete for $ROBOT_IP! 🎉"
echo "================================================================"
