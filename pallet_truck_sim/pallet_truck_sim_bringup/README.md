### Launch standard robot:

`roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch`

### Launch another robot once one is running:

`roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch`

`roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch id_robot:=pallet_truck_2 x_init_pose:=-5 run_simulation_environment:=false`

### Launch simulation environment and two robots independently:

`roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch run_robot:=false`

`roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch run_simulation_environment:=false`

`roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch id_robot:=pallet_truck_2 x_init_pose:=-5 run_simulation_environment:=false`

 
### TODO:

1. Solve bugs in simulation (imu not being subscribed, bad odometry)
1. ~~Split pallet_truck_complete.launch and move the corresponding parts to pallet_truck_gazebo.~~
1. Split spawn_robot.launch from gazebo into run gazebo environment and run robot.
