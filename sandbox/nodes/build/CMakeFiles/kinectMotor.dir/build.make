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
CMAKE_SOURCE_DIR = /home/tesfamic/fuerte_workspace/sandbox/nodes

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tesfamic/fuerte_workspace/sandbox/nodes/build

# Include any dependencies generated for this target.
include CMakeFiles/kinectMotor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/kinectMotor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/kinectMotor.dir/flags.make

CMakeFiles/kinectMotor.dir/src/kinectMotor.o: CMakeFiles/kinectMotor.dir/flags.make
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: ../src/kinectMotor.cpp
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: ../manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/pcl/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/rosbag/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/rosservice/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/kinectMotor.dir/src/kinectMotor.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tesfamic/fuerte_workspace/sandbox/nodes/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/kinectMotor.dir/src/kinectMotor.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/kinectMotor.dir/src/kinectMotor.o -c /home/tesfamic/fuerte_workspace/sandbox/nodes/src/kinectMotor.cpp

CMakeFiles/kinectMotor.dir/src/kinectMotor.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinectMotor.dir/src/kinectMotor.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/tesfamic/fuerte_workspace/sandbox/nodes/src/kinectMotor.cpp > CMakeFiles/kinectMotor.dir/src/kinectMotor.i

CMakeFiles/kinectMotor.dir/src/kinectMotor.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinectMotor.dir/src/kinectMotor.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/tesfamic/fuerte_workspace/sandbox/nodes/src/kinectMotor.cpp -o CMakeFiles/kinectMotor.dir/src/kinectMotor.s

CMakeFiles/kinectMotor.dir/src/kinectMotor.o.requires:
.PHONY : CMakeFiles/kinectMotor.dir/src/kinectMotor.o.requires

CMakeFiles/kinectMotor.dir/src/kinectMotor.o.provides: CMakeFiles/kinectMotor.dir/src/kinectMotor.o.requires
	$(MAKE) -f CMakeFiles/kinectMotor.dir/build.make CMakeFiles/kinectMotor.dir/src/kinectMotor.o.provides.build
.PHONY : CMakeFiles/kinectMotor.dir/src/kinectMotor.o.provides

CMakeFiles/kinectMotor.dir/src/kinectMotor.o.provides.build: CMakeFiles/kinectMotor.dir/src/kinectMotor.o

# Object files for target kinectMotor
kinectMotor_OBJECTS = \
"CMakeFiles/kinectMotor.dir/src/kinectMotor.o"

# External object files for target kinectMotor
kinectMotor_EXTERNAL_OBJECTS =

../bin/kinectMotor: CMakeFiles/kinectMotor.dir/src/kinectMotor.o
../bin/kinectMotor: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
../bin/kinectMotor: CMakeFiles/kinectMotor.dir/build.make
../bin/kinectMotor: CMakeFiles/kinectMotor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/kinectMotor"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinectMotor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/kinectMotor.dir/build: ../bin/kinectMotor
.PHONY : CMakeFiles/kinectMotor.dir/build

CMakeFiles/kinectMotor.dir/requires: CMakeFiles/kinectMotor.dir/src/kinectMotor.o.requires
.PHONY : CMakeFiles/kinectMotor.dir/requires

CMakeFiles/kinectMotor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/kinectMotor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/kinectMotor.dir/clean

CMakeFiles/kinectMotor.dir/depend:
	cd /home/tesfamic/fuerte_workspace/sandbox/nodes/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tesfamic/fuerte_workspace/sandbox/nodes /home/tesfamic/fuerte_workspace/sandbox/nodes /home/tesfamic/fuerte_workspace/sandbox/nodes/build /home/tesfamic/fuerte_workspace/sandbox/nodes/build /home/tesfamic/fuerte_workspace/sandbox/nodes/build/CMakeFiles/kinectMotor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/kinectMotor.dir/depend

