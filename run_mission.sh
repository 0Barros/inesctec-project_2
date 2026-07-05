#!/bin/bash

# Clean up any residual zombie ports/instances first
echo "[SHELL] Cleaning up environment ports..."
pkill -9 -f arducopter
pkill -9 -f mavproxy
pkill -9 -f flyMission.py

# Activate your project virtual environment
echo "[SHELL] Activating Virtual Environment..."
source /home/barros/Documents/project_2/venv/bin/activate

# Launch the ArduCopter SITL Map environment at Dragao
echo "[SHELL] Starting ArduCopter SITL map environment..."
echo "[SHELL] NOTE: Once initialized, wait for 'EKF3 is using GPS' before running the python script!"
cd /home/barros/Documents/project_2/ardupilot/ArduCopter
sim_vehicle.py --console --map -L Dragao
