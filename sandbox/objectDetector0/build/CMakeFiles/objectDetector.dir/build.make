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
CMAKE_SOURCE_DIR = /home/tesfamic/fuerte_workspace/sandbox/objectDetector0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tesfamic/fuerte_workspace/sandbox/objectDetector0/build

# Include any dependencies generated for this target.
include CMakeFiles/objectDetector.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/objectDetector.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/objectDetector.dir/flags.make

CMakeFiles/objectDetector.dir/src/objectDetector.o: CMakeFiles/objectDetector.dir/flags.make
CMakeFiles/objectDetector.dir/src/objectDetector.o: ../src/objectDetector.cpp
CMakeFiles/objectDetector.dir/src/objectDetector.o: ../manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/objectDetector.dir/src/objectDetector.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tesfamic/fuerte_workspace/sandbox/objectDetector0/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/objectDetector.dir/src/objectDetector.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/objectDetector.dir/src/objectDetector.o -c /home/tesfamic/fuerte_workspace/sandbox/objectDetector0/src/objectDetector.cpp

CMakeFiles/objectDetector.dir/src/objectDetector.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/objectDetector.dir/src/objectDetector.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/tesfamic/fuerte_workspace/sandbox/objectDetector0/src/objectDetector.cpp > CMakeFiles/objectDetector.dir/src/objectDetector.i

CMakeFiles/objectDetector.dir/src/objectDetector.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/objectDetector.dir/src/objectDetector.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/tesfamic/fuerte_workspace/sandbox/objectDetector0/src/objectDetector.cpp -o CMakeFiles/objectDetector.dir/src/objectDetector.s

CMakeFiles/objectDetector.dir/src/objectDetector.o.requires:
.PHONY : CMakeFiles/objectDetector.dir/src/objectDetector.o.requires

CMakeFiles/objectDetector.dir/src/objectDetector.o.provides: CMakeFiles/objectDetector.dir/src/objectDetector.o.requires
	$(MAKE) -f CMakeFiles/objectDetector.dir/build.make CMakeFiles/objectDetector.dir/src/objectDetector.o.provides.build
.PHONY : CMakeFiles/objectDetector.dir/src/objectDetector.o.provides

CMakeFiles/objectDetector.dir/src/objectDetector.o.provides.build: CMakeFiles/objectDetector.dir/src/objectDetector.o

# Object files for target objectDetector
objectDetector_OBJECTS = \
"CMakeFiles/objectDetector.dir/src/objectDetector.o"

# External object files for target objectDetector
objectDetector_EXTERNAL_OBJECTS =

../bin/objectDetector: CMakeFiles/objectDetector.dir/src/objectDetector.o
../bin/objectDetector: CMakeFiles/objectDetector.dir/build.make
../bin/objectDetector: CMakeFiles/objectDetector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/objectDetector"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/objectDetector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/objectDetector.dir/build: ../bin/objectDetector
.PHONY : CMakeFiles/objectDetector.dir/build

CMakeFiles/objectDetector.dir/requires: CMakeFiles/objectDetector.dir/src/objectDetector.o.requires
.PHONY : CMakeFiles/objectDetector.dir/requires

CMakeFiles/objectDetector.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/objectDetector.dir/cmake_clean.cmake
.PHONY : CMakeFiles/objectDetector.dir/clean

CMakeFiles/objectDetector.dir/depend:
	cd /home/tesfamic/fuerte_workspace/sandbox/objectDetector0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tesfamic/fuerte_workspace/sandbox/objectDetector0 /home/tesfamic/fuerte_workspace/sandbox/objectDetector0 /home/tesfamic/fuerte_workspace/sandbox/objectDetector0/build /home/tesfamic/fuerte_workspace/sandbox/objectDetector0/build /home/tesfamic/fuerte_workspace/sandbox/objectDetector0/build/CMakeFiles/objectDetector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/objectDetector.dir/depend

