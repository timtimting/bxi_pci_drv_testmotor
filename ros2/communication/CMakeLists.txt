cmake_minimum_required(VERSION 3.8)
# Accept the legacy Python option used by existing ROS build instructions.
if(DEFINED PYTHON_EXECUTABLE)
  if(DEFINED Python3_EXECUTABLE AND NOT "${Python3_EXECUTABLE}" STREQUAL "${PYTHON_EXECUTABLE}")
    message(FATAL_ERROR "PYTHON_EXECUTABLE and Python3_EXECUTABLE must select the same interpreter")
  endif()
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE}")
endif()
project(communication)
# Shared colcon argument applies only to the motor driver.
foreach(option ETHERLAB_ROOT ETHERLAB_INCLUDE_DIR ETHERLAB_LIBRARY)
  if(DEFINED ${option})
    message(STATUS "${option}='${${option}}' applies to bxi_motor_control")
  endif()
endforeach()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCmds.msg"
  "msg/ActuatorStates.msg"
  "msg/BatteryStates.msg"
  "msg/CANFD.msg"
  "msg/CANFDPacket.msg"
  "msg/MotionCommands.msg"
  "msg/StateEstimator.msg"
  "msg/TouchSensor.msg"
  "srv/GaitSwitch.srv"
  "srv/RobotReset.srv"
  "srv/SimulationReset.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
