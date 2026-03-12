#!/bin/bash
# setup_remote_robot.sh
# Run this script from the Control PC to configure a remote TurtleBot3 Raspberry Pi.
#
# Usage: ./setup_remote_robot.sh [ROBOT_IP] [USERNAME (optional)] [ROS_DOMAIN_ID (optional)]
# Example: ./setup_remote_robot.sh 192.168.1.100 ubuntu 42

# --- Safety & Configuration ---
set -e
set -u
set -o pipefail

# --- Argument Parsing ---
if [ -z "$1" ]; then
    echo "Usage: ./setup_remote_robot.sh <ROBOT_IP> [USERNAME (default: ubuntu)] [ROS_DOMAIN_ID (default: 116)]"
    echo "Example: ./setup_remote_robot.sh 192.168.1.15 ubuntu 42"
    exit 1
fi

ROBOT_IP=$1
USERNAME=${2:-ubuntu}
ROS_DOMAIN_ID=${3:-116}
LOCAL_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
LOCAL_SCRIPT_PATH="$LOCAL_SCRIPT_DIR/setup_robot.sh"
REMOTE_TEMP_SCRIPT_PATH="/home/$USERNAME/setup_robot_tmp.sh"

# --- Main Script ---
echo "================================================================="
echo " Deploying TurtleBot3 Setup Script to Remote Robot"
echo "================================================================="
echo "  ‣ Target Robot IP:    $ROBOT_IP"
echo "  ‣ SSH Username:       $USERNAME"
echo "  ‣ ROS_DOMAIN_ID:      $ROS_DOMAIN_ID"
echo "-----------------------------------------------------------------"
read -p "Press [Enter] to continue, or [Ctrl+C] to abort."

# 1. Check if the local setup_robot.sh exists
if [ ! -f "$LOCAL_SCRIPT_PATH" ]; then
    echo "[ERROR] Could not find 'setup_robot.sh' in the script's directory: $LOCAL_SCRIPT_DIR"
    exit 1
fi

# 2. Copy the setup script to the remote robot via SCP
echo
echo "[1/3] Copying setup script to the robot..."
scp "$LOCAL_SCRIPT_PATH" "$USERNAME@$ROBOT_IP:$REMOTE_TEMP_SCRIPT_PATH"

if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to connect or copy files to the robot via SCP."
    echo "        Please check the IP address, username, and your network connection."
    exit 1
fi

# 3. Execute the script on the remote robot via SSH
echo "[2/3] Executing setup script remotely..."
echo "      (You will likely be prompted for the sudo password for '$USERNAME')"
# The -t flag allocates a pseudo-terminal, which is required for sudo password prompts.
# We pass the ROS_DOMAIN_ID as an argument to the remote script.
ssh -t "$USERNAME@$ROBOT_IP" "chmod +x $REMOTE_TEMP_SCRIPT_PATH && sudo $REMOTE_TEMP_SCRIPT_PATH $ROS_DOMAIN_ID"

# 4. Clean up the temporary script from the remote robot
echo "[3/3] Cleaning up temporary files from the robot..."
ssh "$USERNAME@$ROBOT_IP" "rm $REMOTE_TEMP_SCRIPT_PATH"

echo "================================================================="
echo " Remote Setup Complete for $ROBOT_IP! 🎉"
echo "================================================================="
echo "The robot Pi should now be rebooted to ensure all changes take effect."
echo "================================================================="
