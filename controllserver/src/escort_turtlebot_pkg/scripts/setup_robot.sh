#!/bin/bash
# setup_robot.sh
# Run this script directly on the TurtleBot3 Raspberry Pi to configure it for the Escort Project.

# 에러 발생 시 즉시 스크립트 중단 (set -e), 파이프 에러도 잡기 (pipefail)
set -e
set -o pipefail

echo "================================================="
echo " Starting TurtleBot3 Escort Pi Setup Script "
echo "================================================="

# 1. Timezone & Synchronization (Chrony)
echo "[1/4] Setting up Timezone and Chrony..."
sudo timedatectl set-timezone Asia/Seoul
echo "    -> Updating package list..."
if ! sudo apt-get update; then
    echo "Error: apt-get update failed. Check your network connection."
    exit 1
fi
sudo apt-get install -y chrony
sudo systemctl restart chrony
sudo chronyc makestep

# 2. Network & Middleware (CycloneDDS)
echo "[2/4] Installing CycloneDDS Middleware..."
sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp

# 3. Environment Variables (~/.bashrc)
echo "[3/4] Configuring ~/.bashrc Environment Variables..."
# Prevent duplicate entries by removing old ones first
sed -i '/ROS_DOMAIN_ID/d' ~/.bashrc
sed -i '/RMW_IMPLEMENTATION/d' ~/.bashrc
sed -i '/LDS_MODEL/d' ~/.bashrc

echo 'export ROS_DOMAIN_ID=116' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc

# 4. Patching the LiDAR frame_id for namespaces
echo "[4/4] Patching robot.launch.py for Namespaced LiDAR (base_scan)..."
LAUNCH_FILE=~/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py

if [ -f "$LAUNCH_FILE" ]; then
    # Replace 'frame_id': 'base_scan' with 'frame_id': [namespace, '/base_scan']
    sed -i "s/'frame_id': 'base_scan',/'frame_id': \[namespace, '\/base_scan'\],/g" "$LAUNCH_FILE"
    echo " -> Successfully patched: $LAUNCH_FILE"
else
    echo " -> [Warning] Could not find $LAUNCH_FILE."
    echo "    Make sure you have cloned the 'turtlebot3' workspace locally on this Pi."
fi

echo "================================================="
echo " Setup Complete! 🎉"
echo " Please reboot the Pi or run: source ~/.bashrc"
echo "================================================="
