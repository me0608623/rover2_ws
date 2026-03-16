# campusrover calibration

1. Make sure the wheels pressure are both 20

2. start calibration
```=
cr_sensor
cr_start_drive calibration_mode:=true
```
3. input the distance(meter) you want to move
4. Set to release mode（press start and then press Y to release） 

5. pull the cart to the distance that you are setting
6. rotate 180 angles  and repeat (5.)
7. you will get the wheel diameter when you pull the cart 4 times

8. clockwise the cart 360*5 angles
9. counterclockwise the cart 360*5 angles
10. you will get the wheelbase when you rotation the cart 4 times

11. replace the parameter that you get in yaml.

# calibration verify

1. make sure the cart around is big enough

2. start verify
```
cr_sensor
roslaunch campusrover_driver control_validation.launch online_test:=true
```
3. After enter the last command press A to let it move automatic

4. If the final position is same as original posotion,your parameter is prerry correct!
