#roscore &
ps &
sleep 5
source devel/setup.bash\
killall manager&\
sleep 5
fg&\
killall self_driving&\
sleep 5
fg&\
killall streamdata&\
sleep 5
fg&\
sleep 5
rosrun seldat_robot manager&\
sleep 5
rosrun seldat_robot self_driving&\
sleep 5
rosrun seldat_robot streamdata\
