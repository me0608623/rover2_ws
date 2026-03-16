source ~/rover_ws/install/setup.bash
source ~/rover2_ws/install/setup.bash

alias cr_enable_mpc='ros2 service call /enable_bubble_mpc std_srvs/srv/SetBool "data: 1"'
alias cr_demo_launch_bubble="ros2 launch bubble_planner demo_launch.py use_bubble:=true"
alias cr_demo_launch_dwa="ros2 launch bubble_planner demo_launch.py use_bubble:=false"

alias cr_demo_node="python3 ~/rover_ws/src/bubble_planner/demo_campusrover_bubblempc.py"
alias cr_bubble_mpc="ros2 launch bubble_planner mpc.launch.py simulator_mpc:=false"

alias rviz2_navigation_ndt="rviz2 -d ~/rover_ws/src/bubble_planner/rviz/navigation_ndt.rviz"

copy_bubble_to_xavier () {
  rsync -avz --progress \
    --exclude='bubble_mpc/bubble_mpc_model/__pycache__/' \
    --exclude='bubble_mpc/bubble_mpc_model/c_generated_code/' \
    --exclude='bubble_mpc/bubble_mpc_model/acados_ocp.json' \
    --exclude='rviz/' \
    --exclude='.vscode/' \
    --exclude='log/' \
    -e ssh \
    "$HOME/rover_ws/src/bubble_planner/" \
    xavier@192.168.3.10:~/humble_ws/rover2_ws/src/bubble_planner/
}
