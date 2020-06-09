cmake_minimum_required(VERSION 2.8.3)
project(pallet_truck_navigation)

find_package(catkin REQUIRED COMPONENTS
  teb_local_planner
  move_base
)

catkin_package( CATKIN_DEPENDS )

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
