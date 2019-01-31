#roscore &
source devel/setup.bash
#roslaunch seldat_robot slam_2.launch model:="`rospack find seldat_robot`/urdf/robot_seldat2_nov01.urdf"
#roslaunch seldat_robot slam_2_nov01_2.launch model:="`rospack find seldat_robot`/urdf/robot_seldat2_nov01.urdf"
roslaunch seldat_robot slam_2_nov01_2_norviz.launch model:="`rospack find seldat_robot`/urdf/robot_seldat_v5.urdf"
