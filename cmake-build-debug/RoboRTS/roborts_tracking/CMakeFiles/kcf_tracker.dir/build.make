# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /home/cx/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/212.5457.51/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/cx/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/212.5457.51/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cx/brainnavi/testRobot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cx/brainnavi/testRobot/src/cmake-build-debug

# Include any dependencies generated for this target.
include RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/depend.make
# Include the progress variables for this target.
include RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/progress.make

# Include the compile flags for this target's objects.
include RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/flags.make

RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.o: RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/flags.make
RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.o: ../RoboRTS/roborts_tracking/KCFcpp/src/fhog.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cx/brainnavi/testRobot/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.o"
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.o -c /home/cx/brainnavi/testRobot/src/RoboRTS/roborts_tracking/KCFcpp/src/fhog.cpp

RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.i"
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cx/brainnavi/testRobot/src/RoboRTS/roborts_tracking/KCFcpp/src/fhog.cpp > CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.i

RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.s"
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cx/brainnavi/testRobot/src/RoboRTS/roborts_tracking/KCFcpp/src/fhog.cpp -o CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.s

RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.o: RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/flags.make
RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.o: ../RoboRTS/roborts_tracking/KCFcpp/src/kcftracker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cx/brainnavi/testRobot/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.o"
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.o -c /home/cx/brainnavi/testRobot/src/RoboRTS/roborts_tracking/KCFcpp/src/kcftracker.cpp

RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.i"
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cx/brainnavi/testRobot/src/RoboRTS/roborts_tracking/KCFcpp/src/kcftracker.cpp > CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.i

RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.s"
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cx/brainnavi/testRobot/src/RoboRTS/roborts_tracking/KCFcpp/src/kcftracker.cpp -o CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.s

# Object files for target kcf_tracker
kcf_tracker_OBJECTS = \
"CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.o" \
"CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.o"

# External object files for target kcf_tracker
kcf_tracker_EXTERNAL_OBJECTS =

/home/cx/brainnavi/testRobot/devel/lib/libkcf_tracker.a: RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/fhog.cpp.o
/home/cx/brainnavi/testRobot/devel/lib/libkcf_tracker.a: RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/KCFcpp/src/kcftracker.cpp.o
/home/cx/brainnavi/testRobot/devel/lib/libkcf_tracker.a: RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/build.make
/home/cx/brainnavi/testRobot/devel/lib/libkcf_tracker.a: RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cx/brainnavi/testRobot/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library /home/cx/brainnavi/testRobot/devel/lib/libkcf_tracker.a"
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking && $(CMAKE_COMMAND) -P CMakeFiles/kcf_tracker.dir/cmake_clean_target.cmake
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kcf_tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/build: /home/cx/brainnavi/testRobot/devel/lib/libkcf_tracker.a
.PHONY : RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/build

RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/clean:
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking && $(CMAKE_COMMAND) -P CMakeFiles/kcf_tracker.dir/cmake_clean.cmake
.PHONY : RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/clean

RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/depend:
	cd /home/cx/brainnavi/testRobot/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cx/brainnavi/testRobot/src /home/cx/brainnavi/testRobot/src/RoboRTS/roborts_tracking /home/cx/brainnavi/testRobot/src/cmake-build-debug /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking /home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : RoboRTS/roborts_tracking/CMakeFiles/kcf_tracker.dir/depend

