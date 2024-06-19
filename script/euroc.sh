source /home/ares/catkin_rslam/src/cmake-build-debug/devel/setup.bash

#{
#gnome-terminal -t "vins" -- bash -c "roslaunch vins vins_rviz.launch"
#}&

#{
#gnome-terminal -t "vins" -- bash -c "roscore;exec bash;"
#}&

#sleep 2s
#{
## gnome-terminal -t "poslam_node" -- bash -c "rosrun --prefix \"gdb -ex run --args\" poslam poslam_node 8 /vins_estimator/odometry /image_0 /vins_estimator/point_cloud $1 $2;exec bash;"
#gnome-terminal -t "poslam_node" -- bash -c "rosrun poslam poslam_node 8 /vins_estimator/camera_pose /image_0 /vins_estimator/point_cloud $1 $3;exec bash;"
#}&

{
gnome-terminal -t "vins_node" -- bash -c "rosrun vins vins_node /home/ares/Workspace/advio_ws/src/ADVIO2ROSBAG/data/advio-01/iphone/euroc_mono_imu_config.yaml;exec bash;"
}

sleep 2s

{
gnome-terminal -t "run rosbag" -- bash -c "rosbag play /home/ares/Workspace/advio_ws/src/ADVIO2ROSBAG/data/advio-01/iphone/iphone.bag;exec bash;"
}
