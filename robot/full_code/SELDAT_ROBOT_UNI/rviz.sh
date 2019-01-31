#roscore &
source devel/setup.bash
roslaunch seldat_robot rviz.launch model:="`rospack find seldat_robot`/urdf/robot_seldat_v5.urdf"
