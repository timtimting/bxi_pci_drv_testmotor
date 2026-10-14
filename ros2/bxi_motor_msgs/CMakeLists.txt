cmake_minimum_required(VERSION 3.8)
# Accept the legacy Python option used by existing ROS build instructions.
if(DEFINED PYTHON_EXECUTABLE)
  if(DEFINED Python3_EXECUTABLE AND NOT "${Python3_EXECUTABLE}" STREQUAL "${PYTHON_EXECUTABLE}")
    message(FATAL_ERROR "PYTHON_EXECUTABLE and Python3_EXECUTABLE must select the same interpreter")
  endif()
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE}")
endif()
project(bxi_motor_msgs)
# colcon forwards common options to both packages; only the driver links EtherLab.
if(DEFINED ETHERLAB_ROOT)
  message(STATUS "EtherLab prefix '${ETHERLAB_ROOT}' applies to bxi_motor_control; message generation needs no EtherLab")
endif()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg" "msg/MotorState.msg" "msg/MotorStateArray.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
