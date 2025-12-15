cmake_minimum_required(VERSION 3.8)
project(slg_bt_plugins)

find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)

add_library(slg_bt_plugins SHARED
  src/is_stop_gesture_condition.cpp
)

ament_target_dependencies(slg_bt_plugins
  nav2_behavior_tree
  pluginlib
  rclcpp
  std_msgs
)

target_include_directories(slg_bt_plugins PUBLIC include)

pluginlib_export_plugin_description_file(
  nav2_behavior_tree
  plugin_description.xml
)

install(TARGETS slg_bt_plugins
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)
