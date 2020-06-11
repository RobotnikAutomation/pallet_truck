# pallet_truck

Packages for the HR Recycler Autonomous Pallet Truck.

See [pallet_truck_common/README.md](pallet_truck_common/README.md) for common packages.

See [pallet_truck_sim/README.md](pallet_truck_sim/README.md) for simulation packages.

# How to use

Install external dependencies, located at [pallet_truck_common/external](pallet_truck_common/external).
Install external dependencies, located at [pallet_truck/binaries](pallet_truck/binaries).
` sudo dpkg -i binaries/ros-melodic-robotnik-navigation-msgs_0.0.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-robotnik-msgs_1.0.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-joint-read-command-controller_0.13.2-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-ackermann-drive-controller_0.0.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-rcomponent_1.1.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-robotnik-docker_1.0.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-robotnik-move_1.0.0-0bionic_amd64.deb `

Install with submodules.

` git clone --recurse-submodules https://github.com/RobotnikAutomation/pallet_truck.git `

Remove this folder
` pallet_truck/modules/hector_gazebo/hector_gazebo_thermal_camera `
or add a CATKIN_IGNORE file inside. 
