# campusrover hardware configuration files
* [campusrover-devices.rules](campusrover-devices.rules) - Define USB setting
* [setRule.sh](setRule.sh) - A script files to put "campusrover-devies.rules" and ".conf" to system

## Find USB device serial devpath
```
udevadm info -a -n /dev/ttyUSB1 | grep '{devpath}'
udevadm info -a -n /dev/ttyUSB1 | grep '{serial}'
```

## Udev Setting    
Setting the device rule. 
```
cd ~/rover_ws/src/campusrover_base/campusrover_hardware_rule/
sh setRule.sh 
```
