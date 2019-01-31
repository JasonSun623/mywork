#roscore &
source devel/setup.bash
#roslaunch seldat_robot navigation.launch model:="`rospack find seldat_robot`/urdf/robot_seldat2_nov01.urdf"

roslaunch seldat_robot navigation_norviz.launch model:="`rospack find seldat_robot`/urdf/robot_seldat_v5.urdf"

#roslaunch seldat_robot navigation_norviz.launch
