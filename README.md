# rb_ares

Packages for the Autonomous RB Ares.

See [rb_ares_common/README.md](rb_ares_common/README.md) for common packages.

See [rb_ares_sim/README.md](rb_ares_sim/README.md) for simulation packages.

# How to use

Install external dependencies, located at [rb_ares_common/external](rb_ares_common/external).
Install external dependencies, located at [rb_ares/binaries](rb_ares/binaries).
` sudo dpkg -i binaries/ros-melodic-robotnik-navigation-msgs_0.0.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-robotnik-msgs_1.0.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-joint-read-command-controller_0.13.2-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-ackermann-drive-controller_0.0.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-rcomponent_1.1.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-robotnik-docker_1.0.0-0bionic_amd64.deb `

` sudo dpkg -i binaries/ros-melodic-robotnik-move_1.0.0-0bionic_amd64.deb `

Install with submodules.

` git clone --recurse-submodules https://github.com/RobotnikAutomation/rb_ares.git `

Remove this folder
` rb_ares/modules/hector_gazebo/hector_gazebo_thermal_camera `
or add a CATKIN_IGNORE file inside. 
