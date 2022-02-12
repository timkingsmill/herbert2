#!/bin/bash
echo "remap the device serial port(ttyUSBX) to  odrive0 and odrive0"
echo "usb connection as /dev/odrive0 and /dev/odrive1, check it using the command : ls -l /dev | grep odrive"

_current_working_dir="$(pwd)"
cd ~

# Find and enter the ros2 herbert2_robot directory
colcon_cd herbert2_robot

# Install the udev rules file.
echo "start copy herbert2-odrive.rules to /etc/udev/rules.d/"
sudo cp ./scripts/*odrive.rules /etc/udev/rules.d

cd "$_current_working_dir"
unset _current_working_dir

echo " "
echo "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart

echo "All finished. No errors detected"