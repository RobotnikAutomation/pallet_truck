# rb_ares_common

Common packages of the HR Recycler Autonomous Pallet Truck: URDF description of the HR Recycler Autonomous Pallet Truck, control algorithms, platform messages and other files for simulation.


## Packages

### rb_ares_description

The urdf, meshes, and other elements needed in the description are contained here. The standard camera configurations have been included (w/wo sphere_camera, w/wo axis_camera, etc.). The package includes also some launch files to publish the robot state and to test the urdf files in rviz.

### rb_ares_control

This package contains the launch and configuration files to spawn the joint controllers with the ROS controller_manager. Basic controllers are an ackermann_steering_controller and elevator_controller.

### rb_ares_navigation

This package contains the configuration for running the Navigation stack of the rb_ares robot (ROS Navigation, Robotnik Navigation, etc) (Read rb_ares_navigation/README.md)

### rb_ares_localization

This package contains the configuration for running AMCL and map_server.
