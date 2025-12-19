cmake_minimum_required(VERSION 3.8)
project(slg_bt_plugins)

find_package(ament_cmake REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(slg_bt_plugins SHARED
  src/is_stop_gesture_condition.cpp
  src/is_face_detected_condition.cpp
  src/turn_toward_face_action.cpp
  src/fgs_to_bt_node.cpp
  src/a_node_registration.cpp
)

# THIS IS THE KEY LINE TO MAKE THE BT_RegisterNodesFromPlugin SYMBOL VISIBLE
# see https://chatgpt.com/s/t_6941bc0282808191b0b911baf8efea69
target_compile_definitions(slg_bt_plugins PRIVATE BT_PLUGIN_EXPORT)

target_link_libraries(slg_bt_plugins PUBLIC
  ${nav2_behavior_tree_TARGETS}
  ${behaviortree_cpp_TARGETS}
  ${pluginlib_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
)

target_include_directories(slg_bt_plugins PUBLIC include)

pluginlib_export_plugin_description_file(
  nav2_behavior_tree
  plugin_description.xml
)

install(TARGETS slg_bt_plugins
  LIBRARY DESTINATION lib
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  slg_bt_plugins
  slg_bt_plugins
)

ament_package()
