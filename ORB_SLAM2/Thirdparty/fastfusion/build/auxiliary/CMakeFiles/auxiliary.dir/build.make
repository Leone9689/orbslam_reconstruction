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

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build

# Include any dependencies generated for this target.
include auxiliary/CMakeFiles/auxiliary.dir/depend.make

# Include the progress variables for this target.
include auxiliary/CMakeFiles/auxiliary.dir/progress.make

# Include the compile flags for this target's objects.
include auxiliary/CMakeFiles/auxiliary.dir/flags.make

auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o: auxiliary/CMakeFiles/auxiliary.dir/flags.make
auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o: ../auxiliary/debug.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/auxiliary.dir/debug.cpp.o -c /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/debug.cpp

auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/auxiliary.dir/debug.cpp.i"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/debug.cpp > CMakeFiles/auxiliary.dir/debug.cpp.i

auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/auxiliary.dir/debug.cpp.s"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/debug.cpp -o CMakeFiles/auxiliary.dir/debug.cpp.s

auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o.requires:
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o.requires

auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o.provides: auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o.requires
	$(MAKE) -f auxiliary/CMakeFiles/auxiliary.dir/build.make auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o.provides.build
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o.provides

auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o.provides.build: auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o

auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o: auxiliary/CMakeFiles/auxiliary.dir/flags.make
auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o: ../auxiliary/memory.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/auxiliary.dir/memory.cpp.o -c /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/memory.cpp

auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/auxiliary.dir/memory.cpp.i"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/memory.cpp > CMakeFiles/auxiliary.dir/memory.cpp.i

auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/auxiliary.dir/memory.cpp.s"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/memory.cpp -o CMakeFiles/auxiliary.dir/memory.cpp.s

auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o.requires:
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o.requires

auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o.provides: auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o.requires
	$(MAKE) -f auxiliary/CMakeFiles/auxiliary.dir/build.make auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o.provides.build
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o.provides

auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o.provides.build: auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o

auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o: auxiliary/CMakeFiles/auxiliary.dir/flags.make
auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o: ../auxiliary/threadpool.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/auxiliary.dir/threadpool.cpp.o -c /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/threadpool.cpp

auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/auxiliary.dir/threadpool.cpp.i"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/threadpool.cpp > CMakeFiles/auxiliary.dir/threadpool.cpp.i

auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/auxiliary.dir/threadpool.cpp.s"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/threadpool.cpp -o CMakeFiles/auxiliary.dir/threadpool.cpp.s

auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o.requires:
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o.requires

auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o.provides: auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o.requires
	$(MAKE) -f auxiliary/CMakeFiles/auxiliary.dir/build.make auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o.provides.build
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o.provides

auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o.provides.build: auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o

auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o: auxiliary/CMakeFiles/auxiliary.dir/flags.make
auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o: ../auxiliary/plywriter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/auxiliary.dir/plywriter.cpp.o -c /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/plywriter.cpp

auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/auxiliary.dir/plywriter.cpp.i"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/plywriter.cpp > CMakeFiles/auxiliary.dir/plywriter.cpp.i

auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/auxiliary.dir/plywriter.cpp.s"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/plywriter.cpp -o CMakeFiles/auxiliary.dir/plywriter.cpp.s

auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o.requires:
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o.requires

auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o.provides: auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o.requires
	$(MAKE) -f auxiliary/CMakeFiles/auxiliary.dir/build.make auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o.provides.build
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o.provides

auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o.provides.build: auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o

auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o: auxiliary/CMakeFiles/auxiliary.dir/flags.make
auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o: ../auxiliary/ocv_tools.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/auxiliary.dir/ocv_tools.cpp.o -c /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/ocv_tools.cpp

auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/auxiliary.dir/ocv_tools.cpp.i"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/ocv_tools.cpp > CMakeFiles/auxiliary.dir/ocv_tools.cpp.i

auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/auxiliary.dir/ocv_tools.cpp.s"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary/ocv_tools.cpp -o CMakeFiles/auxiliary.dir/ocv_tools.cpp.s

auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o.requires:
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o.requires

auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o.provides: auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o.requires
	$(MAKE) -f auxiliary/CMakeFiles/auxiliary.dir/build.make auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o.provides.build
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o.provides

auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o.provides.build: auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o

# Object files for target auxiliary
auxiliary_OBJECTS = \
"CMakeFiles/auxiliary.dir/debug.cpp.o" \
"CMakeFiles/auxiliary.dir/memory.cpp.o" \
"CMakeFiles/auxiliary.dir/threadpool.cpp.o" \
"CMakeFiles/auxiliary.dir/plywriter.cpp.o" \
"CMakeFiles/auxiliary.dir/ocv_tools.cpp.o"

# External object files for target auxiliary
auxiliary_EXTERNAL_OBJECTS =

../lib/libauxiliary.so: auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o
../lib/libauxiliary.so: auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o
../lib/libauxiliary.so: auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o
../lib/libauxiliary.so: auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o
../lib/libauxiliary.so: auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o
../lib/libauxiliary.so: auxiliary/CMakeFiles/auxiliary.dir/build.make
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_nonfree.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_nonfree.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../lib/libauxiliary.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../lib/libauxiliary.so: auxiliary/CMakeFiles/auxiliary.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library ../../lib/libauxiliary.so"
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/auxiliary.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
auxiliary/CMakeFiles/auxiliary.dir/build: ../lib/libauxiliary.so
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/build

auxiliary/CMakeFiles/auxiliary.dir/requires: auxiliary/CMakeFiles/auxiliary.dir/debug.cpp.o.requires
auxiliary/CMakeFiles/auxiliary.dir/requires: auxiliary/CMakeFiles/auxiliary.dir/memory.cpp.o.requires
auxiliary/CMakeFiles/auxiliary.dir/requires: auxiliary/CMakeFiles/auxiliary.dir/threadpool.cpp.o.requires
auxiliary/CMakeFiles/auxiliary.dir/requires: auxiliary/CMakeFiles/auxiliary.dir/plywriter.cpp.o.requires
auxiliary/CMakeFiles/auxiliary.dir/requires: auxiliary/CMakeFiles/auxiliary.dir/ocv_tools.cpp.o.requires
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/requires

auxiliary/CMakeFiles/auxiliary.dir/clean:
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary && $(CMAKE_COMMAND) -P CMakeFiles/auxiliary.dir/cmake_clean.cmake
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/clean

auxiliary/CMakeFiles/auxiliary.dir/depend:
	cd /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/auxiliary /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary /home/iopenlink/catkin_ws/src/orbslam2_ros/ORB_SLAM2/Thirdparty/fastfusion/build/auxiliary/CMakeFiles/auxiliary.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : auxiliary/CMakeFiles/auxiliary.dir/depend
