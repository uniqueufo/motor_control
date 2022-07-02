include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/asio/include)

# Add executables
add_executable(test_serial_card test_serial_communication_card.cpp)
target_link_libraries(test_serial_card our_hw_sdk asio Threads::Threads yaml-cpp)

add_executable(test_motor
        test_motor_cmd.cpp
#        ../../../src/hal/user_hw/motor_cfg/pan_tilt_motor_msg_process.h
#        ../../../src/hal/user_hw/pan_tilt_motor.cpp
#        ../../../src/hal/user_hw/pan_tilt_motor.h
#        ../../../src/hal/user_hw/pan_tilt_motor_manager.cpp
#        ../../../src/hal/user_hw/pan_tilt_motor_manager.h
#        ../../../src/hal/user_hw/motor_cfg/RMDL_RS485_def.h
        )
target_link_libraries(test_motor our_hw_sdk asio Threads::Threads yaml-cpp)


