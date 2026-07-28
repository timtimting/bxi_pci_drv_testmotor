cmake_minimum_required(VERSION 2.8)
project(bxi_pci_drv_motor_console)

add_compile_options(-Wall -fdiagnostics-color=auto -fPIC -Wno-unused-function)

include_directories(./src ./src/motor ./lib)

set(BXI_PCI_CTL_LIB "${CMAKE_SOURCE_DIR}/lib/libbxi_pci_drv.a")

add_executable(motor_console ./src/main.c ./src/motor/bxi_motor_comm.c)

target_link_libraries(motor_console
  PRIVATE
  pthread
  "${BXI_PCI_CTL_LIB}"
)

set_property(TARGET motor_console APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)
