# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/super/Documents/DifferentialDrive/AprilTags

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/super/Documents/DifferentialDrive/AprilTags/build

# Include any dependencies generated for this target.
include example/CMakeFiles/imu.dir/depend.make

# Include the progress variables for this target.
include example/CMakeFiles/imu.dir/progress.make

# Include the compile flags for this target's objects.
include example/CMakeFiles/imu.dir/flags.make

example/CMakeFiles/imu.dir/imu.cpp.o: example/CMakeFiles/imu.dir/flags.make
example/CMakeFiles/imu.dir/imu.cpp.o: ../example/imu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/super/Documents/DifferentialDrive/AprilTags/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object example/CMakeFiles/imu.dir/imu.cpp.o"
	cd /home/super/Documents/DifferentialDrive/AprilTags/build/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu.dir/imu.cpp.o -c /home/super/Documents/DifferentialDrive/AprilTags/example/imu.cpp

example/CMakeFiles/imu.dir/imu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu.dir/imu.cpp.i"
	cd /home/super/Documents/DifferentialDrive/AprilTags/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/super/Documents/DifferentialDrive/AprilTags/example/imu.cpp > CMakeFiles/imu.dir/imu.cpp.i

example/CMakeFiles/imu.dir/imu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu.dir/imu.cpp.s"
	cd /home/super/Documents/DifferentialDrive/AprilTags/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/super/Documents/DifferentialDrive/AprilTags/example/imu.cpp -o CMakeFiles/imu.dir/imu.cpp.s

example/CMakeFiles/imu.dir/imu.cpp.o.requires:

.PHONY : example/CMakeFiles/imu.dir/imu.cpp.o.requires

example/CMakeFiles/imu.dir/imu.cpp.o.provides: example/CMakeFiles/imu.dir/imu.cpp.o.requires
	$(MAKE) -f example/CMakeFiles/imu.dir/build.make example/CMakeFiles/imu.dir/imu.cpp.o.provides.build
.PHONY : example/CMakeFiles/imu.dir/imu.cpp.o.provides

example/CMakeFiles/imu.dir/imu.cpp.o.provides.build: example/CMakeFiles/imu.dir/imu.cpp.o


example/CMakeFiles/imu.dir/Serial.cpp.o: example/CMakeFiles/imu.dir/flags.make
example/CMakeFiles/imu.dir/Serial.cpp.o: ../example/Serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/super/Documents/DifferentialDrive/AprilTags/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object example/CMakeFiles/imu.dir/Serial.cpp.o"
	cd /home/super/Documents/DifferentialDrive/AprilTags/build/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu.dir/Serial.cpp.o -c /home/super/Documents/DifferentialDrive/AprilTags/example/Serial.cpp

example/CMakeFiles/imu.dir/Serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu.dir/Serial.cpp.i"
	cd /home/super/Documents/DifferentialDrive/AprilTags/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/super/Documents/DifferentialDrive/AprilTags/example/Serial.cpp > CMakeFiles/imu.dir/Serial.cpp.i

example/CMakeFiles/imu.dir/Serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu.dir/Serial.cpp.s"
	cd /home/super/Documents/DifferentialDrive/AprilTags/build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/super/Documents/DifferentialDrive/AprilTags/example/Serial.cpp -o CMakeFiles/imu.dir/Serial.cpp.s

example/CMakeFiles/imu.dir/Serial.cpp.o.requires:

.PHONY : example/CMakeFiles/imu.dir/Serial.cpp.o.requires

example/CMakeFiles/imu.dir/Serial.cpp.o.provides: example/CMakeFiles/imu.dir/Serial.cpp.o.requires
	$(MAKE) -f example/CMakeFiles/imu.dir/build.make example/CMakeFiles/imu.dir/Serial.cpp.o.provides.build
.PHONY : example/CMakeFiles/imu.dir/Serial.cpp.o.provides

example/CMakeFiles/imu.dir/Serial.cpp.o.provides.build: example/CMakeFiles/imu.dir/Serial.cpp.o


# Object files for target imu
imu_OBJECTS = \
"CMakeFiles/imu.dir/imu.cpp.o" \
"CMakeFiles/imu.dir/Serial.cpp.o"

# External object files for target imu
imu_EXTERNAL_OBJECTS =

bin/imu: example/CMakeFiles/imu.dir/imu.cpp.o
bin/imu: example/CMakeFiles/imu.dir/Serial.cpp.o
bin/imu: example/CMakeFiles/imu.dir/build.make
bin/imu: lib/libapriltags.a
bin/imu: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_tracking3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_reg3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_plot3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_dnn3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_text3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_face3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_viz3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_superres3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_shape3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_video3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_photo3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_ml3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_flann3.so.3.1.0
bin/imu: /opt/ros/kinetic/lib/libopencv_core3.so.3.1.0
bin/imu: example/CMakeFiles/imu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/super/Documents/DifferentialDrive/AprilTags/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/imu"
	cd /home/super/Documents/DifferentialDrive/AprilTags/build/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/CMakeFiles/imu.dir/build: bin/imu

.PHONY : example/CMakeFiles/imu.dir/build

example/CMakeFiles/imu.dir/requires: example/CMakeFiles/imu.dir/imu.cpp.o.requires
example/CMakeFiles/imu.dir/requires: example/CMakeFiles/imu.dir/Serial.cpp.o.requires

.PHONY : example/CMakeFiles/imu.dir/requires

example/CMakeFiles/imu.dir/clean:
	cd /home/super/Documents/DifferentialDrive/AprilTags/build/example && $(CMAKE_COMMAND) -P CMakeFiles/imu.dir/cmake_clean.cmake
.PHONY : example/CMakeFiles/imu.dir/clean

example/CMakeFiles/imu.dir/depend:
	cd /home/super/Documents/DifferentialDrive/AprilTags/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/super/Documents/DifferentialDrive/AprilTags /home/super/Documents/DifferentialDrive/AprilTags/example /home/super/Documents/DifferentialDrive/AprilTags/build /home/super/Documents/DifferentialDrive/AprilTags/build/example /home/super/Documents/DifferentialDrive/AprilTags/build/example/CMakeFiles/imu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/CMakeFiles/imu.dir/depend

