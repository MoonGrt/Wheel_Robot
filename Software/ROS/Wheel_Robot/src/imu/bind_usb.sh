#!/bin/bash
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="imu"' > /etc/udev/rules.d/imu.rules

service udev reload
sleep 2
service udev restart