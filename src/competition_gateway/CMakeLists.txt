cmake_minimum_required(VERSION 3.8)
project(competition_gateway)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(serial_protocol src/serial_protocol.cpp)
target_include_directories(serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(serial_protocol PUBLIC cxx_std_17)

# 蓝方串口网关: X 原样发送
add_executable(serial_gateway src/serial_gateway.cpp)
target_link_libraries(serial_gateway serial_protocol)
ament_target_dependencies(serial_gateway geometry_msgs rclcpp std_msgs)

# 红方串口网关: 写死位置帧 field_x_m 取反后发送(内部坐标系不变)
add_executable(serial_gateway_red src/serial_gateway_red.cpp)
target_link_libraries(serial_gateway_red serial_protocol)
ament_target_dependencies(serial_gateway_red geometry_msgs rclcpp std_msgs)

install(TARGETS serial_protocol serial_gateway serial_gateway_red
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_serial_protocol test/test_serial_protocol.cpp)
  target_link_libraries(test_serial_protocol serial_protocol)
endif()

ament_package()