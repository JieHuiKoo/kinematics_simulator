# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jiehui/kinematics_simulator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jiehui/kinematics_simulator/build

# Include any dependencies generated for this target.
include CMakeFiles/kinematics_simulator.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/kinematics_simulator.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/kinematics_simulator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kinematics_simulator.dir/flags.make

CMakeFiles/kinematics_simulator.dir/main.cpp.o: CMakeFiles/kinematics_simulator.dir/flags.make
CMakeFiles/kinematics_simulator.dir/main.cpp.o: ../main.cpp
CMakeFiles/kinematics_simulator.dir/main.cpp.o: CMakeFiles/kinematics_simulator.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiehui/kinematics_simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/kinematics_simulator.dir/main.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/kinematics_simulator.dir/main.cpp.o -MF CMakeFiles/kinematics_simulator.dir/main.cpp.o.d -o CMakeFiles/kinematics_simulator.dir/main.cpp.o -c /home/jiehui/kinematics_simulator/main.cpp

CMakeFiles/kinematics_simulator.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinematics_simulator.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiehui/kinematics_simulator/main.cpp > CMakeFiles/kinematics_simulator.dir/main.cpp.i

CMakeFiles/kinematics_simulator.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinematics_simulator.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiehui/kinematics_simulator/main.cpp -o CMakeFiles/kinematics_simulator.dir/main.cpp.s

# Object files for target kinematics_simulator
kinematics_simulator_OBJECTS = \
"CMakeFiles/kinematics_simulator.dir/main.cpp.o"

# External object files for target kinematics_simulator
kinematics_simulator_EXTERNAL_OBJECTS =

kinematics_simulator: CMakeFiles/kinematics_simulator.dir/main.cpp.o
kinematics_simulator: CMakeFiles/kinematics_simulator.dir/build.make
kinematics_simulator: /usr/local/lib/libopencv_gapi.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_highgui.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_ml.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_objdetect.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_photo.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_stitching.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_video.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_videoio.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_imgcodecs.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_dnn.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_calib3d.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_features2d.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_flann.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_imgproc.so.4.7.0
kinematics_simulator: /usr/local/lib/libopencv_core.so.4.7.0
kinematics_simulator: CMakeFiles/kinematics_simulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jiehui/kinematics_simulator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable kinematics_simulator"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinematics_simulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kinematics_simulator.dir/build: kinematics_simulator
.PHONY : CMakeFiles/kinematics_simulator.dir/build

CMakeFiles/kinematics_simulator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kinematics_simulator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kinematics_simulator.dir/clean

CMakeFiles/kinematics_simulator.dir/depend:
	cd /home/jiehui/kinematics_simulator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jiehui/kinematics_simulator /home/jiehui/kinematics_simulator /home/jiehui/kinematics_simulator/build /home/jiehui/kinematics_simulator/build /home/jiehui/kinematics_simulator/build/CMakeFiles/kinematics_simulator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kinematics_simulator.dir/depend

