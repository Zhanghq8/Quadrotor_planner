# Directed Research
Interactive Planning and Sensing with a Team of Multiple Robotic Vehicles in an Unknown Environment

---

## Dependencies

* hector_quadrotor packages
  * hector_quadrotor(https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor)
  * hector_models(https://github.com/tu-darmstadt-ros-pkg/hector_models)
  * hector_slam(https://github.com/tu-darmstadt-ros-pkg/hector_slam)
  * hector_gazebo(https://github.com/tu-darmstadt-ros-pkg/hector_gazebo)
  * hector_localization(https://github.com/tu-darmstadt-ros-pkg/hector_localization)
  * (OPTIONAL)gazebo_ros_pkgs(https://github.com/ros-simulation/gazebo_ros_pkgs)
  * (OPTIONAL)uuid_msgs(https://github.com/ros-geographic-info/unique_identifier)
* (OPTIONAL)geographic_info ros package(https://github.com/ros-geographic-info/geographic_info)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(linux)
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
* Gazebo >= 7
* Rviz

## Basic Build Instructions

1. Clone hector_* packages into src folder of your workspace
2. Compile: `catkin_make`
3. Clone this repo
4. Compile: `catkin_make`

## Node included:

1. IPAS demo: `ipas_demo`
2. Controller for trajectory tracking: `controller_tracking`
3. Image processing and Map updating: `localmap_update`
4. Take off controller: `take_off_controller`

## How to run:
1. `roslaunch quadrotor_demo spawn_multi_quadrotors.launch`
2. Once the Gazebo and Rviz were launched successfully, `rosrun quadrotor_demo take_off_controller`
3. `roslaunch quadrotor_demo demo.launch`
4. Once quadrotors took off successfully(you will see the msg in the take_off_controller shell), `rosrun quadrotor_demo controller_tracking`
5. Activate the demo with `rostopic pub /updatemap_flag std_msgs/Bool "data: true"`

## Demo:
- <<https://youtu.be/GRI9eFAI1XE>>