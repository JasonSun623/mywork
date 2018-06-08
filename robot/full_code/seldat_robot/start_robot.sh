#!/bin/sh
echo Start Robot
#sleep 10
#roscore &
#sleep 3
#./cmd_cmod_sick.sh
#sudo chmod a+rw /dev/STM32F4 /dev/SICK_LMS200 /dev/LPMS_IMU
#cd ~/seldat_robot/
#source devel/setup.bash
#sleep 3
cd ~/seldat_robot/src/seldat_robot/src
#/etc/udev/rules.d/10-local.rules
#vim /.bashrc 
#===============================
./internet_check.py &
sleep 2
./robot_node_stm_new.py &
sleep 1
./tpid_velocity.py &
sleep 1
#./odom_encoder.py &
./odom.py
sleep 1
#===============================
#./video_publish.py &
#./detect_digits_ros.py &
#./opencv_line_ros.py &
#./red_pub.py &
#./PID_Compute_2.py &
#===============================
rosrun seldat_robot safety & #safety_v2
sleep 1
rosrun seldat_robot twist &
sleep 1
#rosrun lpms_imu lpms_imu_node &
#sleep 3
#roslaunch seldat_robot pose_ekf.launch &
#sleep 2
#./odom_robot.py

#rosrun seldat_robot bridgenode &
#rosrun seldat_robot self_driving &
#rosrun seldat_robot manager

