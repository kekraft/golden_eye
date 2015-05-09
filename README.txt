#To start the camera:
# open terminal
# source the golden eye ws
# run the following commands
cd ~/AppliedRob/golden_eye_ws/src/camera_umd/uvc_camera/launch
~/AppliedRob/golden_eye_ws/src/camera_umd/uvc_camera/launch

# at this point you can start the camera node by doing the following
# start new terminal
# source the golden eye ws
# run the following command
rosrun pong_vision updated_cup_detection.py 


### to get kinect up and running
# run this command
roslaunch freenect_launch freenect.launch

# open up rviz to see it or just bag these files
# if you use rvz change map to be camera optical depth to be frame of reference
/camera/depth/image_rect_raw
/camera/rgb/image_rect_color

# to get the teensie up and communicating with ros
# burn the code onto the teensie using the Arduino IDE
# leave the teensie plugged into the usb port
# in one terminal, start a ros core
roscore
# in another terminal, start the python serial script
# make sure you specify the correct port
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1
# you should be good from there

# to publish from the command line once its running...
# for launcher pid
rostopic pub /launcher/pid_val geometry_msgs/Vector3 -- 0.0 0.0 0.0

# for loader
rostopic pub /loader/load_cmd std_msgs/Bool -- True

# loader switch wire coding
from left to right (looking at the board from usb side to end)
blue, white, green is disconnected 

loader motor wire coding
yellow = positive 
black = negative

power supply to loader coding
black = negative
green = positive

