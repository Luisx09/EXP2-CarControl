cmake_minimum_required(VERSION 2.8.3)
project(lrodriguez-umd)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs rospy sensor_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lrodriguez-umd
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${catkin_INCLUDE_DIRS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/lrodriguez-umd_node.cpp)
add_executable(hello_prime hello.cpp)
target_link_libraries(hello_prime ${catkin_LIBRARIES})

add_executable(rand_pub pubvel.cpp)
target_link_libraries(rand_pub ${catkin_LIBRARIES})

catkin_python_setup()

catkin_install_python(PROGRAMS bin/car_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  catkin_install_python(PROGRAMS bin/img_proc
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

