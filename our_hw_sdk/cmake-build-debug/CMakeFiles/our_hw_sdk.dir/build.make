# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/lql/下载/clion-2021.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/lql/下载/clion-2021.3.4/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/our_hw_sdk.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/our_hw_sdk.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/our_hw_sdk.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/our_hw_sdk.dir/flags.make

CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.o: CMakeFiles/our_hw_sdk.dir/flags.make
CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.o: ../src/communication/async_serial.cpp
CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.o: CMakeFiles/our_hw_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.o -MF CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.o.d -o CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.o -c /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/communication/async_serial.cpp

CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/communication/async_serial.cpp > CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.i

CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/communication/async_serial.cpp -o CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.s

CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.o: CMakeFiles/our_hw_sdk.dir/flags.make
CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.o: ../src/communication/async_can.cpp
CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.o: CMakeFiles/our_hw_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.o -MF CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.o.d -o CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.o -c /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/communication/async_can.cpp

CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/communication/async_can.cpp > CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.i

CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/communication/async_can.cpp -o CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.s

CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.o: CMakeFiles/our_hw_sdk.dir/flags.make
CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.o: ../src/hal/serial_com_motor.cpp
CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.o: CMakeFiles/our_hw_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.o -MF CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.o.d -o CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.o -c /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/serial_com_motor.cpp

CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/serial_com_motor.cpp > CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.i

CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/serial_com_motor.cpp -o CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.s

CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.o: CMakeFiles/our_hw_sdk.dir/flags.make
CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.o: ../src/hal/user_hw/pan_tilt_motor.cpp
CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.o: CMakeFiles/our_hw_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.o -MF CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.o.d -o CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.o -c /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/user_hw/pan_tilt_motor.cpp

CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/user_hw/pan_tilt_motor.cpp > CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.i

CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/user_hw/pan_tilt_motor.cpp -o CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.s

CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.o: CMakeFiles/our_hw_sdk.dir/flags.make
CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.o: ../src/hal/user_hw/pan_tilt_motor_manager.cpp
CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.o: CMakeFiles/our_hw_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.o -MF CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.o.d -o CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.o -c /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/user_hw/pan_tilt_motor_manager.cpp

CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/user_hw/pan_tilt_motor_manager.cpp > CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.i

CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/user_hw/pan_tilt_motor_manager.cpp -o CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.s

CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.o: CMakeFiles/our_hw_sdk.dir/flags.make
CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.o: ../src/hal/serial_device_manager.cpp
CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.o: CMakeFiles/our_hw_sdk.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.o -MF CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.o.d -o CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.o -c /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/serial_device_manager.cpp

CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/serial_device_manager.cpp > CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.i

CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/hal/serial_device_manager.cpp -o CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.s

# Object files for target our_hw_sdk
our_hw_sdk_OBJECTS = \
"CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.o" \
"CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.o" \
"CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.o" \
"CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.o" \
"CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.o" \
"CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.o"

# External object files for target our_hw_sdk
our_hw_sdk_EXTERNAL_OBJECTS =

libour_hw_sdk.a: CMakeFiles/our_hw_sdk.dir/src/communication/async_serial.cpp.o
libour_hw_sdk.a: CMakeFiles/our_hw_sdk.dir/src/communication/async_can.cpp.o
libour_hw_sdk.a: CMakeFiles/our_hw_sdk.dir/src/hal/serial_com_motor.cpp.o
libour_hw_sdk.a: CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor.cpp.o
libour_hw_sdk.a: CMakeFiles/our_hw_sdk.dir/src/hal/user_hw/pan_tilt_motor_manager.cpp.o
libour_hw_sdk.a: CMakeFiles/our_hw_sdk.dir/src/hal/serial_device_manager.cpp.o
libour_hw_sdk.a: CMakeFiles/our_hw_sdk.dir/build.make
libour_hw_sdk.a: CMakeFiles/our_hw_sdk.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX static library libour_hw_sdk.a"
	$(CMAKE_COMMAND) -P CMakeFiles/our_hw_sdk.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/our_hw_sdk.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/our_hw_sdk.dir/build: libour_hw_sdk.a
.PHONY : CMakeFiles/our_hw_sdk.dir/build

CMakeFiles/our_hw_sdk.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/our_hw_sdk.dir/cmake_clean.cmake
.PHONY : CMakeFiles/our_hw_sdk.dir/clean

CMakeFiles/our_hw_sdk.dir/depend:
	cd /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles/our_hw_sdk.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/our_hw_sdk.dir/depend

