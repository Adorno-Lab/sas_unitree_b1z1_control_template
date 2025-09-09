cmake_minimum_required(VERSION 3.8)
project(sas_unitree_b1z1_control_template)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sas_robot_driver_ur REQUIRED)
find_package(sas_robot_driver REQUIRED)
find_package(sas_core REQUIRED)

############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
set(RCLCPP_LOCAL_BINARY_NAME joint_interface_example_cpp)

#add_executable(${RCLCPP_LOCAL_BINARY_NAME}
#   src/joint_interface_example.cpp
#    )

#ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME}
#    rclcpp
#    sas_core
#    sas_robot_driver
#    )

#target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}
#
#    )

#target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>)

#target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC c_std_99 cxx_std_17)

#install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
#    DESTINATION lib/${PROJECT_NAME})

#unset(RCLCPP_LOCAL_BINARY_NAME)

# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

# According to https://github.com/SmartArmStack/sas_robot_driver/blob/ros2/CMakeLists.txt
##### LAUNCH FILES #####

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

##END## LAUNCH FILES #####


##END## Robot config files #####

# According to https://github.com/SmartArmStack/sas_datalogger/blob/78ac681f9cb049c5b9715d215f163565f16b5cdc/CMakeLists.txt
##### PYTHON EXECUTABLES #####

#install(PROGRAMS
#  scripts/joint_interface_example.py
#  DESTINATION lib/${PROJECT_NAME}
#)

##END## PYTHON EXECUTABLES #####

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()    