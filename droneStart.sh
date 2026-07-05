#!/bin/bash

# 1. Activate Virtual Environment
source /home/barros/Documents/project_2/venv_drone/bin/activate

# 2. Go to the ArduCopter directory
cd /home/barros/Documents/project_2/ardupilot/ArduCopter

# 3. Start SITL with console and map
sim_vehicle.py --console --map
