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
CMAKE_SOURCE_DIR = /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build

# Include any dependencies generated for this target.
include src/CMakeFiles/BufferHandler.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/CMakeFiles/BufferHandler.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/BufferHandler.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/BufferHandler.dir/flags.make

src/CMakeFiles/BufferHandler.dir/BufferHandler.cpp.o: src/CMakeFiles/BufferHandler.dir/flags.make
src/CMakeFiles/BufferHandler.dir/BufferHandler.cpp.o: ../src/BufferHandler.cpp
src/CMakeFiles/BufferHandler.dir/BufferHandler.cpp.o: src/CMakeFiles/BufferHandler.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/BufferHandler.dir/BufferHandler.cpp.o"
	cd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/BufferHandler.dir/BufferHandler.cpp.o -MF CMakeFiles/BufferHandler.dir/BufferHandler.cpp.o.d -o CMakeFiles/BufferHandler.dir/BufferHandler.cpp.o -c /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/src/BufferHandler.cpp

src/CMakeFiles/BufferHandler.dir/BufferHandler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BufferHandler.dir/BufferHandler.cpp.i"
	cd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/src/BufferHandler.cpp > CMakeFiles/BufferHandler.dir/BufferHandler.cpp.i

src/CMakeFiles/BufferHandler.dir/BufferHandler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BufferHandler.dir/BufferHandler.cpp.s"
	cd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/src/BufferHandler.cpp -o CMakeFiles/BufferHandler.dir/BufferHandler.cpp.s

# Object files for target BufferHandler
BufferHandler_OBJECTS = \
"CMakeFiles/BufferHandler.dir/BufferHandler.cpp.o"

# External object files for target BufferHandler
BufferHandler_EXTERNAL_OBJECTS =

src/libBufferHandler.a: src/CMakeFiles/BufferHandler.dir/BufferHandler.cpp.o
src/libBufferHandler.a: src/CMakeFiles/BufferHandler.dir/build.make
src/libBufferHandler.a: src/CMakeFiles/BufferHandler.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libBufferHandler.a"
	cd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/src && $(CMAKE_COMMAND) -P CMakeFiles/BufferHandler.dir/cmake_clean_target.cmake
	cd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BufferHandler.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/BufferHandler.dir/build: src/libBufferHandler.a
.PHONY : src/CMakeFiles/BufferHandler.dir/build

src/CMakeFiles/BufferHandler.dir/clean:
	cd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/src && $(CMAKE_COMMAND) -P CMakeFiles/BufferHandler.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/BufferHandler.dir/clean

src/CMakeFiles/BufferHandler.dir/depend:
	cd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/src /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/src /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/src/CMakeFiles/BufferHandler.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/BufferHandler.dir/depend

