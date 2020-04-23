# pallet_truck

Packages for the HR Recycler Autonomous Pallet Truck.

See [pallet_truck_common/README.md](pallet_truck_common/README.md) for common packages.

See [pallet_truck_sim/README.md](pallet_truck_sim/README.md) for simulation packages.

# How to use

Install external dependencies, located at [pallet_truck_common/external](pallet_truck_common/external).
Install external dependencies, located at [pallet_truck/binaries](pallet_truck/binaries).

` sudo dpkg -i binaries/ros-kinetic-ackermann-drive-controller_0.0.0-0xenial_amd64.deb `

` sudo dpkg -i binaries/ros-kinetic-rcomponent_1.1.0-0xenial_amd64.deb `

` sudo dpkg -i binaries/ros-kinetic-robotnik-docker_1.0.0-0xenial_amd64.deb `

` sudo dpkg -i binaries/ros-kinetic-robotnik-move_1.0.0-0xenial_amd64.deb `

` sudo dpkg -i binaries/ros-kinetic-robotnik-navigation-msgs_0.0.0-0xenial_amd64.deb `

Install submodules.

` git clone --recurse-submodules https://github.com/RobotnikAutomation/pallet_truck.git `
