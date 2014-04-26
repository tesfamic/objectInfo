# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tesfamic/fuerte_workspace/sandbox/kinect_motor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tesfamic/fuerte_workspace/sandbox/kinect_motor/build

# Include any dependencies generated for this target.
include CMakeFiles/kinect_aux_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kinect_aux_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kinect_aux_node.dir/flags.make

CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o: CMakeFiles/kinect_aux_node.dir/flags.make
CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o: ../src/kinect_aux_node.cpp
CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o: ../manifest.xml
CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tesfamic/fuerte_workspace/sandbox/kinect_motor/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o -c /home/tesfamic/fuerte_workspace/sandbox/kinect_motor/src/kinect_aux_node.cpp

CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/tesfamic/fuerte_workspace/sandbox/kinect_motor/src/kinect_aux_node.cpp > CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.i

CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/tesfamic/fuerte_workspace/sandbox/kinect_motor/src/kinect_aux_node.cpp -o CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.s

CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o.requires:
.PHONY : CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o.requires

CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o.provides: CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o.requires
	$(MAKE) -f CMakeFiles/kinect_aux_node.dir/build.make CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o.provides.build
.PHONY : CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o.provides

CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o.provides.build: CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o

# Object files for target kinect_aux_node
kinect_aux_node_OBJECTS = \
"CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o"

# External object files for target kinect_aux_node
kinect_aux_node_EXTERNAL_OBJECTS =

../bin/kinect_aux_node: CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o
../bin/kinect_aux_node: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
../bin/kinect_aux_node: CMakeFiles/kinect_aux_node.dir/build.make
../bin/kinect_aux_node: CMakeFiles/kinect_aux_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/kinect_aux_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinect_aux_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kinect_aux_node.dir/build: ../bin/kinect_aux_node
.PHONY : CMakeFiles/kinect_aux_node.dir/build

CMakeFiles/kinect_aux_node.dir/requires: CMakeFiles/kinect_aux_node.dir/src/kinect_aux_node.o.requires
.PHONY : CMakeFiles/kinect_aux_node.dir/requires

CMakeFiles/kinect_aux_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kinect_aux_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kinect_aux_node.dir/clean

CMakeFiles/kinect_aux_node.dir/depend:
	cd /home/tesfamic/fuerte_workspace/sandbox/kinect_motor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tesfamic/fuerte_workspace/sandbox/kinect_motor /home/tesfamic/fuerte_workspace/sandbox/kinect_motor /home/tesfamic/fuerte_workspace/sandbox/kinect_motor/build /home/tesfamic/fuerte_workspace/sandbox/kinect_motor/build /home/tesfamic/fuerte_workspace/sandbox/kinect_motor/build/CMakeFiles/kinect_aux_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kinect_aux_node.dir/depend

