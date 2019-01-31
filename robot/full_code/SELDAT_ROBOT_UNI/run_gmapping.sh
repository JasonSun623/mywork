#roscore &
source devel/setup.bash
#roslaunch seldat_robot navigation.launch model:="`rospack find seldat_robot`/urdf/robot_seldat2_nov01.urdf"
roslaunch seldat_robot gmapping.launch model:="`rospack find seldat_robot`/urdf/robot_seldat_xacro_2.urdf" &
roslaunch seldat_robot navigation_2.launch
