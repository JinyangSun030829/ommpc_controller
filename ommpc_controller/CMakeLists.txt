cmake_minimum_required(VERSION 3.0.2)
project(ommpc_controller)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS "YES")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  rosbag
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/state.cfg
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ommpc_controller
 LIBRARIES minisnap_core
 CATKIN_DEPENDS dynamic_reconfigure geometry_msgs mavros_msgs nav_msgs roscpp rosbag rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

set(SUNRAY_COMMON_LIB_DIR
  "${PROJECT_SOURCE_DIR}/../../../../../General_Module/sunray_common/common_lib"
)

include_directories(
  include
  include/traj_tracking_controller
  ${SUNRAY_COMMON_LIB_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_example_node 
  src/ommpc_example.cpp
)

target_link_libraries(${PROJECT_NAME}_example_node
  ${catkin_LIBRARIES}
  osqp
)

add_library(minisnap_core
  src/trajectory_generator.cpp
  src/trajectory_planner.cpp
)
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
add_dependencies(trajectory_generator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(trajectory_generator_node minisnap_core ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

# Offline calibration only: no ROS master or flight commands.
add_executable(minisnap_circle_probe src/minisnap_circle_probe.cpp)
target_link_libraries(minisnap_circle_probe minisnap_core)

install(TARGETS minisnap_core trajectory_generator_node minisnap_circle_probe
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES include/trajectory_generator.h include/trajectory_planner.h include/stable_hover.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(trajectory_generator_offline_test tests/trajectory_generator_offline_test.cpp)
  target_link_libraries(trajectory_generator_offline_test minisnap_core)
  add_test(NAME minisnap_offline COMMAND trajectory_generator_offline_test)
  add_executable(hover_gate_offline_test tests/hover_gate_offline_test.cpp)
  add_test(NAME hover_gate_offline COMMAND hover_gate_offline_test)
  add_executable(flight_safety_offline_test tests/flight_safety_offline_test.cpp)
  add_test(NAME flight_safety_offline COMMAND flight_safety_offline_test)
endif()

add_executable(traj_tracking_controller_fsm src/traj_tracking_controller_fsm.cpp)
add_dependencies(traj_tracking_controller_fsm
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(traj_tracking_controller_fsm ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_example_node PROPERTIES
    BUILD_RPATH "/usr/local/lib;$ENV{HOME}/.local/lib;/home/yundrone/Sunray/External_Module/osqp/build/out"
)

catkin_install_python(PROGRAMS
  traj/calc_minisnap_circle.py
  traj/fly_cir.py
  traj/dob_realtime_plot.py
  traj/rviz_quad.py
  traj/minisnap_plot.py
  traj/realtime_gauge.py
  traj/resend.py
  traj/plot_mpc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  traj/traj_tracking_fsm.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
