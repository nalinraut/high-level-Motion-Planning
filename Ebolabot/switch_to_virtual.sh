#!/bin/bash
cp Common/system_config_virtual.json Common/system_config.json
echo "Switching system configuration to virtual mode, ready to run kinematic simulation on localhost"
echo "1. Run ./virtual_motion_computer_start.sh"
echo "2. Run ./TaskGUIDemo"