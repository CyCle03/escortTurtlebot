#!/bin/bash
# setup_robot.sh
# Run this script directly on the TurtleBot3 Raspberry Pi to configure it for the Escort Project.
#
# Usage: ./setup_robot.sh [ROS_DOMAIN_ID (optional, default: 116)]
# Example: ./setup_robot.sh 42

# --- Safety & Configuration ---
# Exit immediately if a command exits with a non-zero status.
set -e
# Treat unset variables as an error when substituting.
set -u
# Prevent errors in a pipeline from being masked.
set -o pipefail

# --- Main Script ---
echo "======================================================"
echo " Starting TurtleBot3 Escort Pi Setup Script "
echo "======================================================"
echo "This script is intended for the Raspberry Pi on the TurtleBot3."
echo "Running it on a different machine may have unintended side effects."
echo "======================================================"

# 1. Parameter Handling
ROS_DOMAIN_ID=${1:-116}
echo "[INFO] Using ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# 2. Timezone & Synchronization (Chrony)
echo "[1/4] Setting up Timezone and System Clock (Chrony)..."
sudo timedatectl set-timezone Asia/Seoul
echo "    -> Updating package list (apt-get update)..."
if ! sudo apt-get update; then
    echo "[ERROR] apt-get update failed. Check your network connection and DNS settings."
    exit 1
fi
echo "    -> Installing chrony for time synchronization..."
sudo apt-get install -y chrony
echo "    -> Restarting and forcing clock sync..."
sudo systemctl restart chrony
# The 'makestep' command gradually corrects the time. For an immediate sync,
# 'chronyd -q' is an alternative but can be more disruptive.
sudo chronyc makestep

# 3. Network & Middleware (CycloneDDS)
echo "[2/4] Installing ROS 2 CycloneDDS Middleware..."
sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp

# 4. Environment Variables (~/.bashrc)
echo "[3/4] Configuring ~/.bashrc for ROS Environment..."
BASHRC_FILE=~/.bashrc

# To prevent duplicate entries, we first remove any lines containing these exports.
# This makes the script idempotent (safe to run multiple times).
echo "    -> Removing old environment variable exports..."
sed -i '/export ROS_DOMAIN_ID=/d' "$BASHRC_FILE"
sed -i '/export RMW_IMPLEMENTATION=/d' "$BASHRC_FILE"
sed -i '/export TURTLEBOT3_MODEL=/d' "$BASHRC_FILE"
sed -i '/export LDS_MODEL=/d' "$BASHRC_FILE"

echo "    -> Adding new environment variable exports..."
echo '' >> "$BASHRC_FILE"
echo '# --- Escort Project TurtleBot3 Configuration ---' >> "$BASHRC_FILE"
echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> "$BASHRC_FILE"
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> "$BASHRC_FILE"
# Burger is the most common model, adjust if you have a Waffle.
echo 'export TURTLEBOT3_MODEL=burger' >> "$BASHRC_FILE"
# LDS-02 is the LiDAR model for the Burger.
echo 'export LDS_MODEL=LDS-02' >> "$BASHRC_FILE"
echo '# ---------------------------------------------' >> "$BASHRC_FILE"

# 5. Patching LiDAR frame_id for Namespacing
echo "[4/4] Patching TurtleBot3 launch file for namespaced LiDAR frame..."
# The default launch file hardcodes 'base_scan', which breaks when using namespaces.
# This patch makes it respect the robot's namespace (e.g., 'TB3_1/base_scan').
LAUNCH_FILE=~/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py

if [ -f "$LAUNCH_FILE" ]; then
    if grep -q "'frame_id': \[namespace, '\/base_scan'\]" "$LAUNCH_FILE"; then
        echo "    -> Launch file already appears to be patched. Skipping."
    else
        sed -i "s/'frame_id': 'base_scan',/'frame_id': \[namespace, '\/base_scan'\],/g" "$LAUNCH_FILE"
        echo "    -> Successfully patched: $LAUNCH_FILE"
    fi
else
    echo "    -> [WARNING] Could not find launch file at: $LAUNCH_FILE."
    echo "       Please ensure the 'turtlebot3_ws' is cloned in the home directory on this Pi."
fi

echo "======================================================"
echo " TurtleBot3 Setup Complete! 🎉"
echo "======================================================"
echo "To apply the changes, please either reboot the Pi or run:"
echo "  source ~/.bashrc"
echo "======================================================"
