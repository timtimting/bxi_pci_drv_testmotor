cmake_minimum_required(VERSION 2.8)
project(bxi_pci_drv_test)

add_compile_options(-Wall -fdiagnostics-color=auto -fPIC -Wno-unused-function)

include_directories(./src ./lib)

aux_source_directory(./src DIR_SRC)

set(BXI_PCI_CTL_LIB "${CMAKE_SOURCE_DIR}/lib/libbxi_pci_drv.a")

add_executable(drv_test ./src/drv_test.c)
add_executable(motor_test ./src/motor_test.c ./src/bxi_motor_comm.c ./src/shell/shell.c)
add_executable(motor_flash ./src/motor_flash.c ./src/bxi_motor_comm.c)
add_executable(motor_console ./src/motor_console.c ./src/bxi_motor_comm.c)

target_link_libraries(drv_test
  PRIVATE
  pthread
  "${BXI_PCI_CTL_LIB}"  # 直接链接库的绝对路径
)

target_link_libraries(motor_test
  PRIVATE
  pthread
  "${BXI_PCI_CTL_LIB}"  # 直接链接库的绝对路径
)

target_link_libraries(motor_flash
  PRIVATE
  pthread
  "${BXI_PCI_CTL_LIB}"  # 直接链接库的绝对路径
)

target_link_libraries(motor_console
  PRIVATE
  pthread
  "${BXI_PCI_CTL_LIB}"
)

set_property(TARGET drv_test APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)

set_property(TARGET motor_test APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)

set_property(TARGET motor_flash APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)

set_property(TARGET motor_console APPEND PROPERTY
  OBJECT_DEPENDS "${BXI_PCI_CTL_LIB}"
)
