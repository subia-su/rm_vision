cmake_minimum_required(VERSION 3.10)
project(rm_serial_driver)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/rm_serial_driver.cpp
  src/crc.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_serial_driver::RMSerialDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  enable_testing()
  add_executable(serial_packet_stream_test
    test/test_packet_stream_standalone.cpp
    src/crc.cpp
  )
  target_include_directories(serial_packet_stream_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  # The standalone regression uses assert, including in Release builds.
  target_compile_options(serial_packet_stream_test PRIVATE -UNDEBUG)
  add_test(NAME serial_packet_stream_test COMMAND serial_packet_stream_test)

  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)

