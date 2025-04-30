cmake_minimum_required(VERSION 3.8)
project(final_proyect_as)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  geometry_msgs
  std_msgs
  vision_msgs
  yolo_msgs
  behaviortree_cpp_v3
  ament_index_cpp
  nav2_msgs
  rclcpp_action
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(detectPerson_bt_node SHARED src/final_proyect_as/DetectPersonNode.cpp)
add_library(finish_bt_node SHARED src/final_proyect_as/FinishNode.cpp)
add_library(requestPlayer_bt_node SHARED src/final_proyect_as/RequestPlayersNode.cpp)

list(APPEND plugin_libs

  detectPerson_bt_node
  finish_bt_node
  requestPlayer_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(final_proyect src/main_bt.cpp)
ament_target_dependencies(final_proyect ${dependencies})

install(TARGETS
  ${plugin_libs}
  final_proyect
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  BTs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
