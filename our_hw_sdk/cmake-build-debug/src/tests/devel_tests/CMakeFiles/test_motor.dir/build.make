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
include src/tests/devel_tests/CMakeFiles/test_motor.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/tests/devel_tests/CMakeFiles/test_motor.dir/compiler_depend.make

# Include the progress variables for this target.
include src/tests/devel_tests/CMakeFiles/test_motor.dir/progress.make

# Include the compile flags for this target's objects.
include src/tests/devel_tests/CMakeFiles/test_motor.dir/flags.make

src/tests/devel_tests/CMakeFiles/test_motor.dir/test_motor_cmd.cpp.o: src/tests/devel_tests/CMakeFiles/test_motor.dir/flags.make
src/tests/devel_tests/CMakeFiles/test_motor.dir/test_motor_cmd.cpp.o: ../src/tests/devel_tests/test_motor_cmd.cpp
src/tests/devel_tests/CMakeFiles/test_motor.dir/test_motor_cmd.cpp.o: src/tests/devel_tests/CMakeFiles/test_motor.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/tests/devel_tests/CMakeFiles/test_motor.dir/test_motor_cmd.cpp.o"
	cd /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/src/tests/devel_tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/tests/devel_tests/CMakeFiles/test_motor.dir/test_motor_cmd.cpp.o -MF CMakeFiles/test_motor.dir/test_motor_cmd.cpp.o.d -o CMakeFiles/test_motor.dir/test_motor_cmd.cpp.o -c /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/tests/devel_tests/test_motor_cmd.cpp

src/tests/devel_tests/CMakeFiles/test_motor.dir/test_motor_cmd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_motor.dir/test_motor_cmd.cpp.i"
	cd /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/src/tests/devel_tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/tests/devel_tests/test_motor_cmd.cpp > CMakeFiles/test_motor.dir/test_motor_cmd.cpp.i

src/tests/devel_tests/CMakeFiles/test_motor.dir/test_motor_cmd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_motor.dir/test_motor_cmd.cpp.s"
	cd /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/src/tests/devel_tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/tests/devel_tests/test_motor_cmd.cpp -o CMakeFiles/test_motor.dir/test_motor_cmd.cpp.s

# Object files for target test_motor
test_motor_OBJECTS = \
"CMakeFiles/test_motor.dir/test_motor_cmd.cpp.o"

# External object files for target test_motor
test_motor_EXTERNAL_OBJECTS =

src/tests/devel_tests/test_motor: src/tests/devel_tests/CMakeFiles/test_motor.dir/test_motor_cmd.cpp.o
src/tests/devel_tests/test_motor: src/tests/devel_tests/CMakeFiles/test_motor.dir/build.make
src/tests/devel_tests/test_motor: libour_hw_sdk.a
src/tests/devel_tests/test_motor: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.5.2
src/tests/devel_tests/test_motor: src/tests/devel_tests/CMakeFiles/test_motor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_motor"
	cd /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/src/tests/devel_tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_motor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/tests/devel_tests/CMakeFiles/test_motor.dir/build: src/tests/devel_tests/test_motor
.PHONY : src/tests/devel_tests/CMakeFiles/test_motor.dir/build

src/tests/devel_tests/CMakeFiles/test_motor.dir/clean:
	cd /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/src/tests/devel_tests && $(CMAKE_COMMAND) -P CMakeFiles/test_motor.dir/cmake_clean.cmake
.PHONY : src/tests/devel_tests/CMakeFiles/test_motor.dir/clean

src/tests/devel_tests/CMakeFiles/test_motor.dir/depend:
	cd /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/src/tests/devel_tests /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/src/tests/devel_tests /home/lql/catkin_ws/src/robot_pan/OurRobot/our_hw_sdk/cmake-build-debug/src/tests/devel_tests/CMakeFiles/test_motor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/tests/devel_tests/CMakeFiles/test_motor.dir/depend

