

gnome-terminal -t "mrobot_bringup" -x bash -c "roslaunch mrobot_bringup res.launch;exec bash;"


sleep 2s
gnome-terminal -t "Receptionist" -x bash -c "cd ~/teamwork; python3 Receptionist.py ;"
sleep 2s
gnome-terminal -t "yolov5_ros" -x bash -c "conda activate pytorch ;roslaunch yolov5_ros yolo_v5.launch ;"

