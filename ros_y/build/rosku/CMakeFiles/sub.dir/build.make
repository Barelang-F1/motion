# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/bf1/Music/ros_y/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bf1/Music/ros_y/build

# Include any dependencies generated for this target.
include rosku/CMakeFiles/sub.dir/depend.make

# Include the progress variables for this target.
include rosku/CMakeFiles/sub.dir/progress.make

# Include the compile flags for this target's objects.
include rosku/CMakeFiles/sub.dir/flags.make

rosku/CMakeFiles/sub.dir/src/sub.cpp.o: rosku/CMakeFiles/sub.dir/flags.make
rosku/CMakeFiles/sub.dir/src/sub.cpp.o: /home/bf1/Music/ros_y/src/rosku/src/sub.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bf1/Music/ros_y/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object rosku/CMakeFiles/sub.dir/src/sub.cpp.o"
	cd /home/bf1/Music/ros_y/build/rosku && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sub.dir/src/sub.cpp.o -c /home/bf1/Music/ros_y/src/rosku/src/sub.cpp

rosku/CMakeFiles/sub.dir/src/sub.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sub.dir/src/sub.cpp.i"
	cd /home/bf1/Music/ros_y/build/rosku && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bf1/Music/ros_y/src/rosku/src/sub.cpp > CMakeFiles/sub.dir/src/sub.cpp.i

rosku/CMakeFiles/sub.dir/src/sub.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sub.dir/src/sub.cpp.s"
	cd /home/bf1/Music/ros_y/build/rosku && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bf1/Music/ros_y/src/rosku/src/sub.cpp -o CMakeFiles/sub.dir/src/sub.cpp.s

rosku/CMakeFiles/sub.dir/src/sub.cpp.o.requires:

.PHONY : rosku/CMakeFiles/sub.dir/src/sub.cpp.o.requires

rosku/CMakeFiles/sub.dir/src/sub.cpp.o.provides: rosku/CMakeFiles/sub.dir/src/sub.cpp.o.requires
	$(MAKE) -f rosku/CMakeFiles/sub.dir/build.make rosku/CMakeFiles/sub.dir/src/sub.cpp.o.provides.build
.PHONY : rosku/CMakeFiles/sub.dir/src/sub.cpp.o.provides

rosku/CMakeFiles/sub.dir/src/sub.cpp.o.provides.build: rosku/CMakeFiles/sub.dir/src/sub.cpp.o


# Object files for target sub
sub_OBJECTS = \
"CMakeFiles/sub.dir/src/sub.cpp.o"

# External object files for target sub
sub_EXTERNAL_OBJECTS =

/home/bf1/Music/ros_y/devel/lib/rosku/sub: rosku/CMakeFiles/sub.dir/src/sub.cpp.o
/home/bf1/Music/ros_y/devel/lib/rosku/sub: rosku/CMakeFiles/sub.dir/build.make
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libdynamixel_sdk.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libroscpp.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/librosconsole.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/librostime.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libcpp_common.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libdynamixel_sdk.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libroscpp.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/librosconsole.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/librostime.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /opt/ros/melodic/lib/libcpp_common.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/bf1/Music/ros_y/devel/lib/rosku/sub: /usr/local/lib/libJetsonGPIO.so
/home/bf1/Music/ros_y/devel/lib/rosku/sub: rosku/CMakeFiles/sub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bf1/Music/ros_y/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/bf1/Music/ros_y/devel/lib/rosku/sub"
	cd /home/bf1/Music/ros_y/build/rosku && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sub.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
rosku/CMakeFiles/sub.dir/build: /home/bf1/Music/ros_y/devel/lib/rosku/sub

.PHONY : rosku/CMakeFiles/sub.dir/build

rosku/CMakeFiles/sub.dir/requires: rosku/CMakeFiles/sub.dir/src/sub.cpp.o.requires

.PHONY : rosku/CMakeFiles/sub.dir/requires

rosku/CMakeFiles/sub.dir/clean:
	cd /home/bf1/Music/ros_y/build/rosku && $(CMAKE_COMMAND) -P CMakeFiles/sub.dir/cmake_clean.cmake
.PHONY : rosku/CMakeFiles/sub.dir/clean

rosku/CMakeFiles/sub.dir/depend:
	cd /home/bf1/Music/ros_y/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bf1/Music/ros_y/src /home/bf1/Music/ros_y/src/rosku /home/bf1/Music/ros_y/build /home/bf1/Music/ros_y/build/rosku /home/bf1/Music/ros_y/build/rosku/CMakeFiles/sub.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : rosku/CMakeFiles/sub.dir/depend

