cd workspace/catkin_w/
catkin_make
. devel/setup.bash
. src/wandrian_run/setup.sh
roslaunch wandrian_run environment.launch world_file:=tmp

