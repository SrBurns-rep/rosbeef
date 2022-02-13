gnome-terminal -e "bash -c 'source /opt/ros/noetic/setup.bash && roscore'"
gnome-terminal -e "bash -c 'source /opt/ros/noetic/setup.bash && rosrun rosserial_python serial_node.py /dev/ttyACM0'"
sleep 2s
gnome-terminal -e "bash -c 'source /opt/ros/noetic/setup.bash && rostopic echo chatter'"
sleep 2s
gnome-terminal -e "bash -c 'source /opt/ros/noetic/setup.bash && rostopic pub toggle_led std_msgs/Empty -r1'"