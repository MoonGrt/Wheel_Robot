#!/bin/bash
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", MODE:="0666", GROUP:="dialout", SYMLINK+="motor"' > /etc/udev/rules.d/motor.rules

service udev reload
sleep 2
service udev restart