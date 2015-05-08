#! /bin/bash
# 
# bash script to start the robot as soon as gloin is logged in.
#
# There are a couple things in place to make sure this happens.
# First, Ubuntu is set to log gloin in automatically.
# Second, the script the bash script is set to run on startup via the 
#  ubuntu startup utility with the command "gnome-terminal -e /home/gloin/AppliedRob/golden_eye_ws/extra/initBot.sh > ~/AppliedRob/output.txt"
#
# 
# Kory Kraft
# 4/20/2014
. ~/.bashrc
LOG_FILE=/home/gloin/AppliedRob/golden_eye_ws/extra/init.log

echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
#echo "Running autostart_screens.sh" >> ${LOG_FILE}
echo "Running file" >> ${LOG_FILE}
echo $(date) >> ${LOG_FILE}
echo "#############################################" >> ${LOG_FILE}
echo "" >> ${LOG_FILE}
echo "Logs:" >> ${LOG_FILE}



. /opt/ros/indigo/setup.bash >> ${LOG_FILE}
cd ~/AppliedRob/golden_eye_ws/
source ./devel/setup.sh
roslaunch pong_vision vision.launch >> ${LOG_FILE}
sleep 5
# PATH=$PATH:/opt/arduino-1.6.3/ &
# roscore 
# rqt
# rosrun rviz rviz &
# subl 
# google-chrome-stable 
