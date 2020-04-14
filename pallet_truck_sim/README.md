# pallet_truck_sim

Packages for the simulation of the Autonomous Pallet Truck.

## Packages

### pallet_truck_sim_bringup

Launch files that launch the complete simulation of the robot.

Launch sample simulation with:

```
roslaunch pallet_truck_sim_bringup pallet_truck_complete.launch
```

### pallet_truck_gazebo

Launch files and world files to start the simulation in Gazebo. Should not be used directly but throught the `pallet_truck_sim_bringup` package, unless you want to do something non-standard.
