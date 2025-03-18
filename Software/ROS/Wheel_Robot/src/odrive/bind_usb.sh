#!/bin/bash
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", MODE:="0666", GROUP:="dialout", SYMLINK+="odrive_uart"' > /etc/udev/rules.d/odrive.rules

service udev reload
sleep 2
service udev restart