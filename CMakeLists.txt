cmake_minimum_required(VERSION 2.8)
project(bxi_pci_drv_motor_console)

add_compile_options(-Wall -fdiagnostics-color=auto -fPIC -Wno-unused-function)

include_directories(./src ./src/lib ./lib)

set(BXI_PCI_CTL_LIB "${CMAKE_SOURCE_DIR}/lib/libbxi_pci_drv.a")
set(ETHERLAB_ROOT "" CACHE PATH "EtherLab installation prefix")
option(ENABLE_ETHERLAB "Enable EtherCAT through the EtherLab master" ON)

if(ENABLE_ETHERLAB)
  find_path(ETHERLAB_INCLUDE_DIR
    NAMES ecrt.h
    PATHS "${ETHERLAB_ROOT}" "$ENV{ETHERLAB_ROOT}"
    PATH_SUFFIXES include
  )
  find_library(ETHERLAB_LIBRARY
    NAMES ethercat
    PATHS "${ETHERLAB_ROOT}" "$ENV{ETHERLAB_ROOT}"
    PATH_SUFFIXES lib lib64
  )
endif()

if(ETHERLAB_INCLUDE_DIR AND ETHERLAB_LIBRARY)
  include_directories("${ETHERLAB_INCLUDE_DIR}")
  add_definitions(-DHAVE_ETHERLAB=1)
  message(STATUS "EtherLab master enabled: ${ETHERLAB_LIBRARY}")
else()
  message(STATUS "EtherLab not found: EtherCAT commands will report build instructions")
endif()

add_executable(motor_console ./src/main.c ./src/lib/bxi_motor_comm.c)

target_link_libraries(motor_console
  PRIVATE
  m
  pthread
  "${BXI_PCI_CTL_LIB}"
)

if(ETHERLAB_INCLUDE_DIR AND ETHERLAB_LIBRARY)
  target_link_libraries(motor_console PRIVATE "${ETHERLAB_LIBRARY}")
endif()

set_property(TARGET motor_console APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)
