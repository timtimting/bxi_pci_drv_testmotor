cmake_minimum_required(VERSION 2.8)
project(bxi_pci_drv_motor_console)

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  message(FATAL_ERROR "test/maita_arm requires an ARM64 toolchain for lib/libbxi_pci_drv.a")
endif()

add_compile_options(-Wall -fdiagnostics-color=auto -fPIC -Wno-unused-function)

include_directories(./src ./src/lib ./lib)

set(BXI_PCI_CTL_LIB "${CMAKE_SOURCE_DIR}/lib/libbxi_pci_drv.a")

add_executable(motor_console ./src/main.c ./src/lib/bxi_motor_comm.c)

target_link_libraries(motor_console
  PRIVATE
  pthread
  "${BXI_PCI_CTL_LIB}"
)

set_property(TARGET motor_console APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)
