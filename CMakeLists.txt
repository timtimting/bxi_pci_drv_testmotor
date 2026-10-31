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

add_executable(motor_console ./src/main.c ./src/lib/bxi_motor_comm.c)

target_link_libraries(motor_console
  PRIVATE
  pthread
  m
  "${BXI_PCI_CTL_LIB}"
)

set_property(TARGET motor_console APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)
