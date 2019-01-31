#roscore &
source devel/setup.bash
#roslaunch seldat_robot navigation.launch model:="`rospack find seldat_robot`/urdf/robot_seldat2_nov01.urdf"
#roslaunch seldat_robot navigation.launch model:="`rospack find seldat_robot`/urdf/robot_seldat_xacro.urdf"
#roslaunch seldat_robot navigation.launch
roslaunch seldat_robot navigation_2.launch model:="`rospack find seldat_robot`/urdf/robot_seldat_v5.urdf"
