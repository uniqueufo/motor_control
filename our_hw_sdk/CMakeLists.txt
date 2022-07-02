cmake_minimum_required(VERSION 3.10.2)
project(our_hw_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(default_build_type "Release")

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

# import asio library
add_subdirectory(3rdparty/asio)


add_library(${PROJECT_NAME}
#        src/communication/asyncio/async_serial.cpp
#        src/communication/asyncio/AsyncCanDevice.cpp
#        # agilex mobile platforms
#        src/platforms/mobile_base.cpp
#        src/platforms/scout_base.cpp
#        src/platforms/scout_can_parser.c
#        src/platforms/scout_uart_parser.c
        src/communication/communication_device.hpp
        src/communication/async_listener_interface.hpp
        src/communication/async_serial.cpp
        src/communication/async_serial.h
        src/communication/async_can.cpp
        src/communication/async_can.h

        src/hal/stopwatch.h
        src/hal/protocol/serial_port_common_def.h
        src/hal/user_hw/motor_cfg/RMDL_RS485_def.h
        src/hal/motor.h
        src/hal/serial_com_motor.cpp
        src/hal/serial_com_motor.h
        src/hal/user_hw/motor_cfg/pan_tilt_motor_msg_process.h
        src/hal/user_hw/pan_tilt_motor.cpp
        src/hal/user_hw/pan_tilt_motor.h
        src/hal/user_hw/pan_tilt_motor_manager.cpp
        src/hal/user_hw/pan_tilt_motor_manager.h
        src/hal/serial_device_manager.cpp
        src/hal/serial_device_manager.h
        )

#add_executable(${PROJECT_NAME}
#        #        src/communication/asyncio/async_serial.cpp
#        #        src/communication/asyncio/AsyncCanDevice.cpp
#        #        # agilex mobile platforms
#        #        src/platforms/mobile_base.cpp
#        #        src/platforms/scout_base.cpp
#        #        src/platforms/scout_can_parser.c
#        #        src/platforms/scout_uart_parser.c
#        src/communication/communication_device.hpp
#        src/communication/async_listener_interface.hpp
#        src/communication/async_serial.cpp
#        src/communication/async_serial.h
#        src/communication/async_can.cpp
#        src/communication/async_can.h
#
#        src/hal/stopwatch.h
#        src/hal/protocol/serial_port_common_def.h
#        src/hal/user_hw/motor_cfg/RMDL_RS485_def.h
#        src/hal/motor.h
#        src/hal/serial_com_motor.cpp
#        src/hal/serial_com_motor.h
#        src/hal/user_hw/motor_cfg/pan_tilt_motor_msg_process.h
#        src/hal/user_hw/pan_tilt_motor.cpp
#        src/hal/user_hw/pan_tilt_motor.h
#        src/hal/user_hw/pan_tilt_motor_manager.cpp
#        src/hal/user_hw/pan_tilt_motor_manager.h
#        src/hal/serial_device_manager.cpp
#        src/hal/serial_device_manager.h
#
#        src/tests/devel_tests/test_motor_cmd.cpp
#        )

target_link_libraries(${PROJECT_NAME} asio Threads::Threads yaml-cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PROJECT_SOURCE_DIR}/include
        PUBLIC src)

add_subdirectory(src/tests/devel_tests)
