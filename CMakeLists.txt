cmake_minimum_required(VERSION 3.8)
project(beta_ekf_ros)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(bng_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BetaEkfState.msg"
  "msg/VehicleInputs.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

get_filename_component(
  BETA_EKF_CORE_DIR_DEFAULT
  "$ENV{HOME}/projects/PX4-Drive/src/modules/simple_sub_pub/"
  ABSOLUTE
)
set(
  BETA_EKF_CORE_DIR
  "${BETA_EKF_CORE_DIR_DEFAULT}"
  CACHE PATH
  "Directory containing beta_ekf_casadi.cpp and the generated XAL model"
)

set(BETA_EKF_REQUIRED_FILES
  beta_ekf_casadi.cpp
  beta_ekf_casadi.hpp
  ekf_core.hpp
  generated/xal_ekf_dimensions.h
  generated/xal_ekf_model.c
  generated/xal_ekf_model.h
)
foreach(required_file IN LISTS BETA_EKF_REQUIRED_FILES)
  if(NOT EXISTS "${BETA_EKF_CORE_DIR}/${required_file}")
    message(FATAL_ERROR
      "BETA_EKF_CORE_DIR='${BETA_EKF_CORE_DIR}' is missing ${required_file}")
  endif()
endforeach()

add_library(beta_ekf_core STATIC
  "${BETA_EKF_CORE_DIR}/beta_ekf_casadi.cpp"
  "${BETA_EKF_CORE_DIR}/generated/xal_ekf_model.c"
)
set_target_properties(beta_ekf_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(beta_ekf_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${BETA_EKF_CORE_DIR}>"
    "$<BUILD_INTERFACE:${BETA_EKF_CORE_DIR}/generated>"
)
target_compile_options(beta_ekf_core PRIVATE
  $<$<COMPILE_LANGUAGE:C>:-Wno-double-promotion -Wno-float-equal>
)

add_executable(beta_ekf_node src/beta_ekf_node.cpp)
target_compile_options(beta_ekf_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(beta_ekf_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${BETA_EKF_CORE_DIR}"
)
ament_target_dependencies(beta_ekf_node
  bng_msgs
  builtin_interfaces
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(beta_ekf_node beta_ekf_core "${cpp_typesupport_target}")

install(TARGETS beta_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES include/beta_ekf_ros/frame_conversions.hpp
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
