#sudo chmod a+rw /dev/ttyUSB0 ##ttyUSB0
#roscore &
sleep 5
#rosparam set sicklms/port /dev/ttyUSB0 ##ttyUSB0
#rosparam set sicklms/baud 38400
#rosrun sicktoolbox_wrapper sicklms
rosrun sicktoolbox_wrapper sicklms _port:=/dev/ttyUSB1 _baud:=38400
