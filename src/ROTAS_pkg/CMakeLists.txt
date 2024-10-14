cmake_minimum_required(VERSION 3.5)
project(ROTAS_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(talker src/ros2_talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

if(WIN32)
  message(STATUS "Building for Windows")
  add_executable(relay src/ros2_relay.cpp)
  target_link_libraries(relay ws2_32 ${OpenCV_LIBRARIES})
  ament_target_dependencies(relay rclcpp std_msgs)
  install(TARGETS relay DESTINATION lib/${PROJECT_NAME})
elseif(UNIX)
  message(STATUS "Building for Linux")
  add_executable(relay_linux src/ros2_relay_linux.cpp)
  target_link_libraries(relay_linux ${OpenCV_LIBS})
  ament_target_dependencies(relay_linux rclcpp std_msgs)
  install(TARGETS relay_linux DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS talker DESTINATION lib/${PROJECT_NAME})

ament_package()