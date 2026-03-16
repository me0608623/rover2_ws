source ~/rover2_ws/install/setup.bash
#alias cr_demo="roslaunch campusrover_demo demo.launch"
alias cr_sensor="ros2 launch campusrover_sensors start_sensors_launch.py"
#alias cr_arm="roslaunch campusrover_arm_move campusrover_robot_arm.launch"
#alias cr_camera="roslaunch campusrover_sensors start_camera.launch"
alias cr_start_drive="ros2 launch campusrover_driver campusrover_driver_launch.py"
#alias cr_collect_data="roslaunch campusrover_sensors bag_file.launch"
#alias cr_sync_map="scp -r justin@192.168.3.50:/home/justin/campusrover_maps/ ~/"
#alias cr_move_eeoffice="roslaunch campusrover_demo pub_goal.launch" # goal:=ee_office
#alias cr_bat_service="nohup ~/campusrover_services/bms-rs485.py -time 300 -server 35.229.155.19 -port 8000 -bat 2021-B02 &"
