echo "source ~/rover2_ws/src/campusrover_base/commands.bash" >> ~/.bashrc
source ~/.bashrc
#echo "Setting ROS depands"
#git submodule update --init --recursive
#cd ~/rover_ws
#rosdep install --from-paths src --ignore-src -r -y

echo "install build tool"
#echo "Detected Ubuntu Version: " $(lsb_release -rs)
#if [[ $(lsb_release -rs) == "20.04" ]]; then # replace 8.04 by the number of release you want
#    echo "Intalling catkin-tools for python3"
#    sudo apt-get install python3-catkin-tools -y
#else
#    echo "Intalling catkin-tools for python2"
#    sudo apt-get install python-catkin-tools -y
#fi
#catkin build
echo "Setting harware rules"
cd ~/rover2_ws/src/campusrover_base/campusrover_hardware_rule/
./setRule.sh

cd ~/rover2_ws/src/campusrover_base/campusrover_hardware_rule/
if [[ $(lsb_release -rs) > "20.00" ]]; then # replace 8.04 by the number of release you want
    echo "CAN BUS by systemd-networkd.service"
    ./set_cando_unbuntu20.sh
#else
#    echo "Intalling catkin-tools for python2"
#    sudo sed -i 's/managed=false/managed=true/' /etc/NetworkManager/NetworkManager.conf
#    FILE=/etc/network/interfaces.d/can0
#    if [[ -f "$FILE" ]]; then
#        echo "$FILE exists."
#    else
#        sudo sh -c "echo 'allow-hotplug can0\niface can0 can static\n    bitrate 500000' >> /etc/network/interfaces.d/can0"
#    fi
#    FILE=/etc/network/interfaces
#    LINE="source /etc/network/interfaces.d/*"
#    if grep -q $LINE $FILE; then
#        echo $LINE "... Found"
#        sudo sed -i '2 i\source /etc/network/interfaces.d/*' $FILE
#    fi
#    service networking restart
fi
