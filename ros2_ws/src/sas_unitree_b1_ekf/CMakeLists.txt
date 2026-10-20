cmake_minimum_required(VERSION 3.8)
project(sas_unitree_b1_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_conversions REQUIRED)
find_package(sas_robot_driver REQUIRED)
find_package(sas_tools REQUIRED)

set(RCLCPP_LOCAL_BINARY_NAME sas_unitree_b1_ekf_node)

add_executable(${RCLCPP_LOCAL_BINARY_NAME}
    src/sas_unitree_b1_ekf_node.cpp
    src/sas_unitree_b1_ekf.cpp
    )

ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME}
    rclcpp Eigen3 geometry_msgs tf2 tf2_ros sas_core sas_common sas_conversions sas_robot_driver sas_tools
    )

target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}
    dqrobotics
    )

target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC c_std_99 cxx_std_20)

install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
    DESTINATION lib/${PROJECT_NAME})

unset(RCLCPP_LOCAL_BINARY_NAME)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
