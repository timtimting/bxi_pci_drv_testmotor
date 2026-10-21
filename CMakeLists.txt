cmake_minimum_required(VERSION 2.8)
project(bxi_pci_drv_motor_console)

add_compile_options(-Wall -fdiagnostics-color=auto -fPIC -Wno-unused-function)

include_directories(./src ./src/lib ./lib)

set(BXI_PCI_CTL_LIB "${CMAKE_SOURCE_DIR}/lib/libbxi_pci_drv.a")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(BXI_PCI_CTL_LIB "${CMAKE_SOURCE_DIR}/lib/arm64/libbxi_pci_drv.a")
elseif(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  message(FATAL_ERROR "Unsupported BXI PCI driver architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
if(NOT EXISTS "${BXI_PCI_CTL_LIB}")
  message(FATAL_ERROR "BXI PCI driver library not found: ${BXI_PCI_CTL_LIB}")
endif()
message(STATUS "BXI PCI driver library: ${BXI_PCI_CTL_LIB}")
set(ETHERLAB_ROOT "" CACHE PATH "EtherLab installation prefix")
option(ENABLE_ETHERLAB "Enable EtherCAT through the EtherLab master" ON)
option(REQUIRE_ETHERLAB "Fail configuration when EtherLab is unavailable" OFF)

if(REQUIRE_ETHERLAB AND NOT ENABLE_ETHERLAB)
  message(FATAL_ERROR "REQUIRE_ETHERLAB needs ENABLE_ETHERLAB=ON")
endif()

if(ENABLE_ETHERLAB)
  set(ETHERLAB_SEARCH_ROOTS "${ETHERLAB_ROOT}" "$ENV{ETHERLAB_ROOT}" /usr/local/etherlab /opt/etherlab)
  find_path(ETHERLAB_INCLUDE_DIR
    NAMES ecrt.h
    HINTS ${ETHERLAB_SEARCH_ROOTS}
    PATH_SUFFIXES include
  )
  find_library(ETHERLAB_LIBRARY
    NAMES ethercat
    HINTS ${ETHERLAB_SEARCH_ROOTS}
    PATH_SUFFIXES lib lib64
  )
endif()

if(ENABLE_ETHERLAB AND ETHERLAB_INCLUDE_DIR AND ETHERLAB_LIBRARY)
  include_directories("${ETHERLAB_INCLUDE_DIR}")
  add_definitions(-DHAVE_ETHERLAB=1)
  message(STATUS "EtherLab master enabled: ${ETHERLAB_LIBRARY}")
elseif(REQUIRE_ETHERLAB)
  message(FATAL_ERROR "EtherLab headers (ecrt.h) or libethercat not found. Install the EtherLab development library or set ETHERLAB_ROOT to its installation prefix.")
elseif(ENABLE_ETHERLAB)
  message(WARNING "EtherLab not found: EtherCAT commands will be unavailable. Install the development library and rebuild.")
else()
  message(STATUS "EtherLab master disabled")
endif()

add_executable(motor_console ./src/main.c ./src/lib/bxi_motor_comm.c)
set_target_properties(motor_console PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/internal")
configure_file("${CMAKE_SOURCE_DIR}/run_motor_console.sh" "${CMAKE_BINARY_DIR}/motor_console" COPYONLY)

target_link_libraries(motor_console
  PRIVATE
  m
  pthread
  "${BXI_PCI_CTL_LIB}"
)

if(ENABLE_ETHERLAB AND ETHERLAB_INCLUDE_DIR AND ETHERLAB_LIBRARY)
  target_link_libraries(motor_console PRIVATE "${ETHERLAB_LIBRARY}")
endif()

set_property(TARGET motor_console APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)
