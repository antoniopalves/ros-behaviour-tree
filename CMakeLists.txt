cmake_minimum_required(VERSION 3.5)
project(trsa_bt_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

############################################################
# DEPENDÊNCIAS
############################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav2_bt_navigator REQUIRED)


############################################################
# PLUGIN C++
############################################################
add_library(cone_in_sight_bt_node SHARED
  src/cone_in_sight.cpp
)

ament_target_dependencies(cone_in_sight_bt_node
  rclcpp
  behaviortree_cpp_v3
  std_msgs
)

# <<< ESTA LINHA ERA O QUE FALTAVA
target_include_directories(cone_in_sight_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# >>>

pluginlib_export_plugin_description_file(
  behaviortree_cpp_v3
  resource/bt_plugin.xml
)

install(
  TARGETS cone_in_sight_bt_node
  LIBRARY DESTINATION lib
)

############################################################
# PYTHON NODES
############################################################
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    trsa_bt_plugins/init_pose_publisher.py
    trsa_bt_plugins/cone_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

############################################################
# INSTALL XML / CONFIG
############################################################
install(
  DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

############################################################
# PACKAGE
############################################################
ament_package()
