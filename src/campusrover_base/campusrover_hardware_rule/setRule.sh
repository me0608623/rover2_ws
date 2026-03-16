sudo cp campusrover-devices-chgh.rules /etc/udev/rules.d/campusrover-devices.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
