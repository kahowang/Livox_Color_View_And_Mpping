# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

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
CMAKE_COMMAND = /usr/share/cmake-3.24.0/bin/cmake

# The command to remove a file.
RM = /usr/share/cmake-3.24.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lory/livox_color_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lory/livox_color_ws/src/build

# Utility rule file for livox_color_generate_messages_nodejs.

# Include any custom commands dependencies for this target.
include livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/compiler_depend.make

# Include the progress variables for this target.
include livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/progress.make

livox_color/CMakeFiles/livox_color_generate_messages_nodejs: devel/share/gennodejs/ros/livox_color/srv/save_pose.js
livox_color/CMakeFiles/livox_color_generate_messages_nodejs: devel/share/gennodejs/ros/livox_color/srv/save_map.js

devel/share/gennodejs/ros/livox_color/srv/save_map.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
devel/share/gennodejs/ros/livox_color/srv/save_map.js: /home/lory/livox_color_ws/src/livox_color/srv/save_map.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/lory/livox_color_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating Javascript code from livox_color/save_map.srv"
	cd /home/lory/livox_color_ws/src/build/livox_color && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/lory/livox_color_ws/src/livox_color/srv/save_map.srv -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p livox_color -o /home/lory/livox_color_ws/src/build/devel/share/gennodejs/ros/livox_color/srv

devel/share/gennodejs/ros/livox_color/srv/save_pose.js: /opt/ros/melodic/lib/gennodejs/gen_nodejs.py
devel/share/gennodejs/ros/livox_color/srv/save_pose.js: /home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/lory/livox_color_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Javascript code from livox_color/save_pose.srv"
	cd /home/lory/livox_color_ws/src/build/livox_color && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gennodejs/cmake/../../../lib/gennodejs/gen_nodejs.py /home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p livox_color -o /home/lory/livox_color_ws/src/build/devel/share/gennodejs/ros/livox_color/srv

livox_color_generate_messages_nodejs: devel/share/gennodejs/ros/livox_color/srv/save_map.js
livox_color_generate_messages_nodejs: devel/share/gennodejs/ros/livox_color/srv/save_pose.js
livox_color_generate_messages_nodejs: livox_color/CMakeFiles/livox_color_generate_messages_nodejs
livox_color_generate_messages_nodejs: livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/build.make
.PHONY : livox_color_generate_messages_nodejs

# Rule to build all files generated by this target.
livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/build: livox_color_generate_messages_nodejs
.PHONY : livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/build

livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/clean:
	cd /home/lory/livox_color_ws/src/build/livox_color && $(CMAKE_COMMAND) -P CMakeFiles/livox_color_generate_messages_nodejs.dir/cmake_clean.cmake
.PHONY : livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/clean

livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/depend:
	cd /home/lory/livox_color_ws/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lory/livox_color_ws/src /home/lory/livox_color_ws/src/livox_color /home/lory/livox_color_ws/src/build /home/lory/livox_color_ws/src/build/livox_color /home/lory/livox_color_ws/src/build/livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : livox_color/CMakeFiles/livox_color_generate_messages_nodejs.dir/depend

