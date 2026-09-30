cmake_minimum_required(VERSION 2.8)
project(bxi_pci_drv_motor_console)

add_compile_options(-Wall -fdiagnostics-color=auto -fPIC -Wno-unused-function)

include_directories(./src ./src/lib ./lib)

set(BXI_PCI_CTL_LIB "${CMAKE_SOURCE_DIR}/lib/libbxi_pci_drv.a")
set(SOEM_ROOT "" CACHE PATH "SOEM source/build root, for example $HOME/SOEM-v1.4.0")
option(ENABLE_SOEM "Enable EtherCAT discovery through SOEM when available" ON)

if(ENABLE_SOEM)
  find_path(SOEM_INCLUDE_DIR
    NAMES soem/ethercat.h
    PATHS
      "${SOEM_ROOT}"
      "$ENV{SOEM_ROOT}"
      "$ENV{HOME}/SOEM-v1.4.0"
    PATH_SUFFIXES include
  )
  find_library(SOEM_LIBRARY
    NAMES soem
    PATHS
      "${SOEM_ROOT}/build"
      "${SOEM_ROOT}/build/lib"
      "$ENV{SOEM_ROOT}/build"
      "$ENV{SOEM_ROOT}/build/lib"
      "$ENV{HOME}/SOEM-v1.4.0/build"
      "$ENV{HOME}/SOEM-v1.4.0/build/lib"
    PATH_SUFFIXES lib
  )
endif()

if(SOEM_INCLUDE_DIR AND SOEM_LIBRARY)
  include_directories(
    "${SOEM_INCLUDE_DIR}"
    "${SOEM_INCLUDE_DIR}/soem"
    "${SOEM_INCLUDE_DIR}/osal"
    "${SOEM_INCLUDE_DIR}/osal/linux"
    "${SOEM_INCLUDE_DIR}/oshw"
    "${SOEM_INCLUDE_DIR}/oshw/linux"
  )
  add_definitions(-DHAVE_SOEM=1)
  message(STATUS "SOEM EtherCAT discovery enabled: ${SOEM_LIBRARY}")
else()
  message(STATUS "SOEM not found: ethercat_scan will report the build instruction")
endif()

add_executable(motor_console ./src/main.c ./src/lib/bxi_motor_comm.c)

target_link_libraries(motor_console
  PRIVATE
  m
  pthread
  "${BXI_PCI_CTL_LIB}"
)

if(SOEM_INCLUDE_DIR AND SOEM_LIBRARY)
  target_link_libraries(motor_console PRIVATE "${SOEM_LIBRARY}")
endif()

set_property(TARGET motor_console APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)
