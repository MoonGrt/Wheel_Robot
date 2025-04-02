#!/bin/bash
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", SYMLINK+="imu"' > /etc/udev/rules.d/imu.rules

service udev reload
sleep 2
service udev restart