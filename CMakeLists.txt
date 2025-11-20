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

############################################################
# NÃO TENS QUALQUER PLUGIN C++ ATUALMENTE
# Logo, NÃO deve haver add_library nem ament_target_dependencies
# Nem pluginlib_export_plugin_description_file
############################################################

# REMOVEI COMPLETAMENTE:
# - add_library(wait_action_bt_node ...)
# - ament_target_dependencies(wait_action_bt_node ...)
# - pluginlib_export_plugin_description_file(...)
# - install(TARGETS wait_action_bt_node ...)

############################################################
# PYTHON NODE (init_pose_publisher.py)
############################################################
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    trsa_bt_plugins/init_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME init_pose_publisher
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

install(
  FILES resource/bt_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

############################################################
# PACKAGE
############################################################
ament_package()
