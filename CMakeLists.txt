cmake_minimum_required(VERSION 3.0.2)
project(robotics_odometry_project)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation
    nav_msgs
    std_msgs
    message_filters
    tf
    robotics_hw1
    dynamic_reconfigure
    geometry_msgs
    std_srvs
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

add_message_files(
 FILES
 integratedOdom.msg

 )

add_service_files(
  FILES
  resetOdometry.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    std_srvs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    message_runtime
    message_filters
    robotics_hw1
    tf
    geometry_msgs
    std_srvs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(robot_main_node
  src/robot_node.cpp
)
add_dependencies(robot_main_node custom_messages_generate_messages_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  
)
target_link_libraries(robot_main_node
  ${catkin_LIBRARIES}
)

add_executable(robot_utility_node
  src/utility_node.cpp
)
add_dependencies(robot_utility_node 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
  
)
target_link_libraries(robot_utility_node
  ${catkin_LIBRARIES}
)

add_executable(gt_calibration_node
  src/gt_calibration_node.cpp
)
add_dependencies(gt_calibration_node 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
  
)
target_link_libraries(gt_calibration_node
  ${catkin_LIBRARIES}
)

