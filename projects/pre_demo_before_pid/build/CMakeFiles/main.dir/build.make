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
CMAKE_SOURCE_DIR = /home/ti-dsp/projects/pre_demo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ti-dsp/projects/pre_demo/build

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ti-dsp/projects/pre_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/main.cpp.o -c /home/ti-dsp/projects/pre_demo/main.cpp

CMakeFiles/main.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ti-dsp/projects/pre_demo/main.cpp > CMakeFiles/main.dir/main.cpp.i

CMakeFiles/main.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ti-dsp/projects/pre_demo/main.cpp -o CMakeFiles/main.dir/main.cpp.s

CMakeFiles/main.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/main.cpp.o.requires

CMakeFiles/main.dir/main.cpp.o.provides: CMakeFiles/main.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/main.cpp.o.provides

CMakeFiles/main.dir/main.cpp.o.provides.build: CMakeFiles/main.dir/main.cpp.o


CMakeFiles/main.dir/Jive.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/Jive.cpp.o: ../Jive.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ti-dsp/projects/pre_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/Jive.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/Jive.cpp.o -c /home/ti-dsp/projects/pre_demo/Jive.cpp

CMakeFiles/main.dir/Jive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/Jive.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ti-dsp/projects/pre_demo/Jive.cpp > CMakeFiles/main.dir/Jive.cpp.i

CMakeFiles/main.dir/Jive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/Jive.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ti-dsp/projects/pre_demo/Jive.cpp -o CMakeFiles/main.dir/Jive.cpp.s

CMakeFiles/main.dir/Jive.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/Jive.cpp.o.requires

CMakeFiles/main.dir/Jive.cpp.o.provides: CMakeFiles/main.dir/Jive.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/Jive.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/Jive.cpp.o.provides

CMakeFiles/main.dir/Jive.cpp.o.provides.build: CMakeFiles/main.dir/Jive.cpp.o


CMakeFiles/main.dir/TOFApp.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/TOFApp.cpp.o: ../TOFApp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ti-dsp/projects/pre_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/TOFApp.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/TOFApp.cpp.o -c /home/ti-dsp/projects/pre_demo/TOFApp.cpp

CMakeFiles/main.dir/TOFApp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/TOFApp.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ti-dsp/projects/pre_demo/TOFApp.cpp > CMakeFiles/main.dir/TOFApp.cpp.i

CMakeFiles/main.dir/TOFApp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/TOFApp.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ti-dsp/projects/pre_demo/TOFApp.cpp -o CMakeFiles/main.dir/TOFApp.cpp.s

CMakeFiles/main.dir/TOFApp.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/TOFApp.cpp.o.requires

CMakeFiles/main.dir/TOFApp.cpp.o.provides: CMakeFiles/main.dir/TOFApp.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/TOFApp.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/TOFApp.cpp.o.provides

CMakeFiles/main.dir/TOFApp.cpp.o.provides.build: CMakeFiles/main.dir/TOFApp.cpp.o


CMakeFiles/main.dir/motor.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/motor.cpp.o: ../motor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ti-dsp/projects/pre_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/main.dir/motor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/motor.cpp.o -c /home/ti-dsp/projects/pre_demo/motor.cpp

CMakeFiles/main.dir/motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/motor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ti-dsp/projects/pre_demo/motor.cpp > CMakeFiles/main.dir/motor.cpp.i

CMakeFiles/main.dir/motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/motor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ti-dsp/projects/pre_demo/motor.cpp -o CMakeFiles/main.dir/motor.cpp.s

CMakeFiles/main.dir/motor.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/motor.cpp.o.requires

CMakeFiles/main.dir/motor.cpp.o.provides: CMakeFiles/main.dir/motor.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/motor.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/motor.cpp.o.provides

CMakeFiles/main.dir/motor.cpp.o.provides.build: CMakeFiles/main.dir/motor.cpp.o


CMakeFiles/main.dir/boostxl.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/boostxl.cpp.o: ../boostxl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ti-dsp/projects/pre_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/main.dir/boostxl.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/boostxl.cpp.o -c /home/ti-dsp/projects/pre_demo/boostxl.cpp

CMakeFiles/main.dir/boostxl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/boostxl.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ti-dsp/projects/pre_demo/boostxl.cpp > CMakeFiles/main.dir/boostxl.cpp.i

CMakeFiles/main.dir/boostxl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/boostxl.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ti-dsp/projects/pre_demo/boostxl.cpp -o CMakeFiles/main.dir/boostxl.cpp.s

CMakeFiles/main.dir/boostxl.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/boostxl.cpp.o.requires

CMakeFiles/main.dir/boostxl.cpp.o.provides: CMakeFiles/main.dir/boostxl.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/boostxl.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/boostxl.cpp.o.provides

CMakeFiles/main.dir/boostxl.cpp.o.provides.build: CMakeFiles/main.dir/boostxl.cpp.o


# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/main.cpp.o" \
"CMakeFiles/main.dir/Jive.cpp.o" \
"CMakeFiles/main.dir/TOFApp.cpp.o" \
"CMakeFiles/main.dir/motor.cpp.o" \
"CMakeFiles/main.dir/boostxl.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/main.cpp.o
main: CMakeFiles/main.dir/Jive.cpp.o
main: CMakeFiles/main.dir/TOFApp.cpp.o
main: CMakeFiles/main.dir/motor.cpp.o
main: CMakeFiles/main.dir/boostxl.cpp.o
main: CMakeFiles/main.dir/build.make
main: /usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_ts.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_superres.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_stitching.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_gpu.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_contrib.so.2.4.9
main: /usr/lib/libti3dtof.so
main: /usr/lib/libvoxel.so
main: /usr/lib/libvoxelpcl.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/libpcl_io.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/libpcl_octree.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/arm-linux-gnueabihf/libflann_cpp_s.a
main: /usr/lib/arm-linux-gnueabihf/libflann_cpp_s.a
main: /usr/lib/libpcl_visualization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/libvtkGenericFiltering.so.5.10.1
main: /usr/lib/libvtkGeovis.so.5.10.1
main: /usr/lib/libvtkCharts.so.5.10.1
main: /usr/lib/libpcl_io.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/libpcl_octree.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/arm-linux-gnueabihf/libflann_cpp_s.a
main: /usr/lib/libpcl_kdtree.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/arm-linux-gnueabihf/libflann_cpp_s.a
main: /usr/lib/libpcl_search.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/arm-linux-gnueabihf/libflann_cpp_s.a
main: /usr/lib/libpcl_kdtree.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/libpcl_octree.so
main: /usr/lib/arm-linux-gnueabihf/libboost_system.so
main: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
main: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
main: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
main: /usr/lib/arm-linux-gnueabihf/libboost_iostreams.so
main: /usr/lib/arm-linux-gnueabihf/libboost_serialization.so
main: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
main: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
main: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
main: /usr/lib/arm-linux-gnueabihf/libpthread.so
main: /usr/lib/libpcl_common.so
main: /usr/lib/libpcl_io.so
main: /usr/lib/libpcl_visualization.so
main: /usr/lib/libpcl_search.so
main: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.9
main: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.9
main: /usr/lib/libvtkViews.so.5.10.1
main: /usr/lib/libvtkInfovis.so.5.10.1
main: /usr/lib/libvtkWidgets.so.5.10.1
main: /usr/lib/libvtkVolumeRendering.so.5.10.1
main: /usr/lib/libvtkHybrid.so.5.10.1
main: /usr/lib/libvtkParallel.so.5.10.1
main: /usr/lib/libvtkRendering.so.5.10.1
main: /usr/lib/libvtkImaging.so.5.10.1
main: /usr/lib/libvtkGraphics.so.5.10.1
main: /usr/lib/libvtkIO.so.5.10.1
main: /usr/lib/libvtkFiltering.so.5.10.1
main: /usr/lib/libvtkCommon.so.5.10.1
main: /usr/lib/libvtksys.so.5.10.1
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ti-dsp/projects/pre_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/requires: CMakeFiles/main.dir/main.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/Jive.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/TOFApp.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/motor.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/boostxl.cpp.o.requires

.PHONY : CMakeFiles/main.dir/requires

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/ti-dsp/projects/pre_demo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ti-dsp/projects/pre_demo /home/ti-dsp/projects/pre_demo /home/ti-dsp/projects/pre_demo/build /home/ti-dsp/projects/pre_demo/build /home/ti-dsp/projects/pre_demo/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

