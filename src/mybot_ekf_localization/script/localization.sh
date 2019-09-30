#!bin/bash

gnome-terminal --tab -e "sh -c 'roslaunch mybot_ekf_localization localization.launch'" --tab -e "sh -c 'sleep 10; rqt_plot /position_error/x:y:z'" 
