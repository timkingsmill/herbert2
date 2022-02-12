#!/bin/bash
echo "remap the device serial port(ttyUSBX) to  rplidar"
echo "rplidar usb connection as /dev/rplidar , check it using the command : ls -l /dev|grep ttyUSB"

_current_working_dir="$(pwd)"
cd ~

# Find and enter the ros2 herbert2_rplidar directory
colcon_cd herbert2_rplidar

# Install the udev rules file.
echo "start copy rplidar.rules to /etc/udev/rules.d/"
sudo cp ./scripts/*rplidar.rules /etc/udev/rules.d

cd "$_current_working_dir"
unset _current_working_dir

echo " "
echo "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart

echo "All finished. No errors detected"