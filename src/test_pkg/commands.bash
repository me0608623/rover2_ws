source ~/rover_ws/install/setup.bash

alias cr_dwa_control="python3 ~/rover_ws/src/test_pkg/scripts/dwa_con.py"

copy_test_to_xavier () {
  rsync -avz --progress \
    --exclude='.vscode/' \
    --exclude='log/' \
    --exclude='rviz/' \
    -e ssh \
    "$HOME/rover_ws/src/test_pkg/" \
    xavier@192.168.3.10:~/humble_ws/rover2_ws/src/test_pkg/
}