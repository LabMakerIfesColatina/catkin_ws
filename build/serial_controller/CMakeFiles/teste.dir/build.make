# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/maker/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maker/catkin_ws/build

# Include any dependencies generated for this target.
include serial_controller/CMakeFiles/teste.dir/depend.make

# Include the progress variables for this target.
include serial_controller/CMakeFiles/teste.dir/progress.make

# Include the compile flags for this target's objects.
include serial_controller/CMakeFiles/teste.dir/flags.make

serial_controller/CMakeFiles/teste.dir/src/joy.cpp.o: serial_controller/CMakeFiles/teste.dir/flags.make
serial_controller/CMakeFiles/teste.dir/src/joy.cpp.o: /home/maker/catkin_ws/src/serial_controller/src/joy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maker/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object serial_controller/CMakeFiles/teste.dir/src/joy.cpp.o"
	cd /home/maker/catkin_ws/build/serial_controller && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/teste.dir/src/joy.cpp.o -c /home/maker/catkin_ws/src/serial_controller/src/joy.cpp

serial_controller/CMakeFiles/teste.dir/src/joy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/teste.dir/src/joy.cpp.i"
	cd /home/maker/catkin_ws/build/serial_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maker/catkin_ws/src/serial_controller/src/joy.cpp > CMakeFiles/teste.dir/src/joy.cpp.i

serial_controller/CMakeFiles/teste.dir/src/joy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/teste.dir/src/joy.cpp.s"
	cd /home/maker/catkin_ws/build/serial_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maker/catkin_ws/src/serial_controller/src/joy.cpp -o CMakeFiles/teste.dir/src/joy.cpp.s

# Object files for target teste
teste_OBJECTS = \
"CMakeFiles/teste.dir/src/joy.cpp.o"

# External object files for target teste
teste_EXTERNAL_OBJECTS =

/home/maker/catkin_ws/devel/lib/serial_controller/teste: serial_controller/CMakeFiles/teste.dir/src/joy.cpp.o
/home/maker/catkin_ws/devel/lib/serial_controller/teste: serial_controller/CMakeFiles/teste.dir/build.make
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /opt/ros/noetic/lib/libroscpp.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /opt/ros/noetic/lib/librosconsole.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /opt/ros/noetic/lib/librostime.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /opt/ros/noetic/lib/libcpp_common.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/maker/catkin_ws/devel/lib/serial_controller/teste: /opt/ros/noetic/lib/libserial.so
/home/maker/catkin_ws/devel/lib/serial_controller/teste: serial_controller/CMakeFiles/teste.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maker/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/maker/catkin_ws/devel/lib/serial_controller/teste"
	cd /home/maker/catkin_ws/build/serial_controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/teste.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
serial_controller/CMakeFiles/teste.dir/build: /home/maker/catkin_ws/devel/lib/serial_controller/teste

.PHONY : serial_controller/CMakeFiles/teste.dir/build

serial_controller/CMakeFiles/teste.dir/clean:
	cd /home/maker/catkin_ws/build/serial_controller && $(CMAKE_COMMAND) -P CMakeFiles/teste.dir/cmake_clean.cmake
.PHONY : serial_controller/CMakeFiles/teste.dir/clean

serial_controller/CMakeFiles/teste.dir/depend:
	cd /home/maker/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maker/catkin_ws/src /home/maker/catkin_ws/src/serial_controller /home/maker/catkin_ws/build /home/maker/catkin_ws/build/serial_controller /home/maker/catkin_ws/build/serial_controller/CMakeFiles/teste.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial_controller/CMakeFiles/teste.dir/depend

