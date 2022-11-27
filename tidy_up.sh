# gnome-terminal -t "base" -x bash -c "roslaunch mrobot_bringup res.launch;exec bash;"
# sleep 5
gnome-terminal -t "robot" -x bash -c "roslaunch kinova_bringup kinova_robot.launch;exec bash;"
sleep 1
gnome-terminal -t "pan" -x bash -c "rosrun pan_tilt_driver run_pan.py;exec bash;"
sleep 2s
gnome-terminal -t "moveit" -x bash -c "roslaunch j2s7s300_moveit_config j2s7s300_demo.launch; exec bash;"
sleep 1s
gnome-terminal -t "tidy_up" -x bash -c "cd ~/teamwork; python3 tidy_up.py;exec bash;"
