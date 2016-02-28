cd workspace/catkin_w/
catkin_make
. devel/setup.bash
. src/wandrian_run/setup.sh
roslaunch wandrian_run run.launch plan_name:=spiral_stc starting_point_x:=0.75 starting_point_y:=-0.75 robot_size:=0.5
