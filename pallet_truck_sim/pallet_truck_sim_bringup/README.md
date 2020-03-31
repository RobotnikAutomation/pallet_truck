### Launch standard robot:

roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch

### Launch another robot once one is running:

roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch id_robot:=robot2 x_init_pose:=-5 launch_gazebo:=false

### TODO:

1. Solve bugs in simulation (imu not being subscribed, bad odometry)
1. Split pallet_truck_complete.launch and move the corresponding parts to pallet_truck_gazebo.
