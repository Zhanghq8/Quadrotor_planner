cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_demo)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge 
  image_transport 
  std_srvs
  sensor_msgs 
  camera_info_manager
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)
# find_package(Eigen3 3.1.2)
find_package(Eigen3)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pose.msg
  path.msg
  pathes.msg
  final_path.msg
  localmap.msg
  obstacle_info.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES quadrotor_demo
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add auto_team libraries
set(AUTOTEAM_LIB_SRC
    include/quadrotor_demo/auto_team/auto_team.hpp
    include/quadrotor_demo/auto_team/auto_team_impl.hpp
)

add_library(auto_team INTERFACE)
target_sources(auto_team INTERFACE ${VEHICLE_LIB_SRC})
target_include_directories(auto_team INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

# Add auto_vehicle libraries
set(AUTOVEHICLE_LIB_SRC
    include/quadrotor_demo/auto_vehicle/auto_vehicle.hpp
    include/quadrotor_demo/auto_vehicle/auto_vehicle.cpp 
    include/quadrotor_demo/auto_vehicle/tasks.hpp 
    include/quadrotor_demo/auto_vehicle/tasks.cpp
)

add_library(auto_vehicle STATIC ${AUTOVEHICLE_LIB_SRC})
target_include_directories(auto_vehicle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add graph libraries
set(GRAPH_LIB_SRC
  include/quadrotor_demo/graph/graph.hpp
  include/quadrotor_demo/graph/vertex.hpp
  include/quadrotor_demo/graph/edge.hpp
  include/quadrotor_demo/graph/graph_impl.hpp
  include/quadrotor_demo/graph/vertex_impl.hpp
  include/quadrotor_demo/graph/edge_impl.hpp
  include/quadrotor_demo/graph/helper_func.hpp
  include/quadrotor_demo/graph/priority_queue.hpp
  include/quadrotor_demo/graph/astar.hpp
)
add_library(graph INTERFACE)
target_sources(graph INTERFACE ${GRAPH_LIB_SRC})
target_include_directories(graph INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

# Add ltl libraries
set(LTL_LIB_SRC
    include/quadrotor_demo/ltl/buchi_automaton.cpp
    include/quadrotor_demo/ltl/spot_hoa_interpreter.cpp
    include/quadrotor_demo/ltl/product_automaton.cpp
    include/quadrotor_demo/ltl/buchi_automaton.hpp
    include/quadrotor_demo/ltl/cell_label.hpp
    include/quadrotor_demo/ltl/product_automaton.hpp
    include/quadrotor_demo/ltl/region_label.hpp
    include/quadrotor_demo/ltl/spot_hoa_interpreter.hpp
)

add_library(ltl STATIC ${LTL_LIB_SRC})
target_link_libraries(ltl map spot bddx auto_vehicle)
target_include_directories(ltl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add map libraries
set(MAP_LIB_SRC
  include/quadrotor_demo/map/square_grid.cpp  
  include/quadrotor_demo/map/common_types.hpp
  include/quadrotor_demo/map/grid_cell_impl.hpp
  include/quadrotor_demo/map/square_grid.hpp
)

add_library(map STATIC ${MAP_LIB_SRC})
target_link_libraries(map ltl auto_vehicle)

target_include_directories(map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add taskassignment libraries
set(TASKASSIGNMENT_LIB_SRC
    include/quadrotor_demo/task_assignment/cbba.hpp
    include/quadrotor_demo/task_assignment/cbba_impl.hpp  
)

add_library(taskassignment INTERFACE)
target_sources(taskassignment INTERFACE ${TASKASSIGNMENT_LIB_SRC})
target_include_directories(taskassignment INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/quadrotor_demo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/quadrotor_demo_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_quadrotor_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(task_demo src/task_demo.cpp)
target_link_libraries(task_demo auto_vehicle map  ${catkin_LIBRARIES})
add_executable(ipas_demo src/ipas_demo.cpp)
target_link_libraries(ipas_demo auto_vehicle map  ${catkin_LIBRARIES})
add_executable(take_off_controller src/take_off_controller.cpp)
target_link_libraries(take_off_controller ${catkin_LIBRARIES})
add_executable(controller_tracking src/controller_tracking.cpp)
target_link_libraries(controller_tracking ${catkin_LIBRARIES})
add_executable(camera_detection src/camera_detection.cpp)
target_link_libraries(camera_detection ${catkin_LIBRARIES})
add_executable(display_camera_view src/display_camera_view.cpp)
target_link_libraries(display_camera_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(localmap_update src/localmap_update.cpp)
target_link_libraries(localmap_update ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
