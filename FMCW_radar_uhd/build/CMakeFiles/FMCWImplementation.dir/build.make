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
include CMakeFiles/FMCWImplementation.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/FMCWImplementation.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/FMCWImplementation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FMCWImplementation.dir/flags.make

CMakeFiles/FMCWImplementation.dir/main.cpp.o: CMakeFiles/FMCWImplementation.dir/flags.make
CMakeFiles/FMCWImplementation.dir/main.cpp.o: ../main.cpp
CMakeFiles/FMCWImplementation.dir/main.cpp.o: CMakeFiles/FMCWImplementation.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FMCWImplementation.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/FMCWImplementation.dir/main.cpp.o -MF CMakeFiles/FMCWImplementation.dir/main.cpp.o.d -o CMakeFiles/FMCWImplementation.dir/main.cpp.o -c /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/main.cpp

CMakeFiles/FMCWImplementation.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FMCWImplementation.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/main.cpp > CMakeFiles/FMCWImplementation.dir/main.cpp.i

CMakeFiles/FMCWImplementation.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FMCWImplementation.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/main.cpp -o CMakeFiles/FMCWImplementation.dir/main.cpp.s

# Object files for target FMCWImplementation
FMCWImplementation_OBJECTS = \
"CMakeFiles/FMCWImplementation.dir/main.cpp.o"

# External object files for target FMCWImplementation
FMCWImplementation_EXTERNAL_OBJECTS =

FMCWImplementation: CMakeFiles/FMCWImplementation.dir/main.cpp.o
FMCWImplementation: CMakeFiles/FMCWImplementation.dir/build.make
FMCWImplementation: src/libJSONHandler.a
FMCWImplementation: src/libUSRPHandler.a
FMCWImplementation: /usr/local/lib/libuhd.so
FMCWImplementation: src/libBufferHandler.a
FMCWImplementation: src/libRADAR.a
FMCWImplementation: src/libATTACKER.a
FMCWImplementation: src/sensing_subsystem/libSpectrogramHandler.a
FMCWImplementation: src/sensing_subsystem/libEnergyDetector.a
FMCWImplementation: src/libFMCWHandler.a
FMCWImplementation: src/libRADAR.a
FMCWImplementation: src/libATTACKER.a
FMCWImplementation: src/sensing_subsystem/libSensingSubsystem.a
FMCWImplementation: src/attacking_subsystem/libAttackingSubsystem.a
FMCWImplementation: src/libUSRPHandler.a
FMCWImplementation: src/sensing_subsystem/libSpectrogramHandler.a
FMCWImplementation: src/sensing_subsystem/libEnergyDetector.a
FMCWImplementation: /usr/local/lib/libuhd.so
FMCWImplementation: src/libBufferHandler.a
FMCWImplementation: CMakeFiles/FMCWImplementation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable FMCWImplementation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FMCWImplementation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FMCWImplementation.dir/build: FMCWImplementation
.PHONY : CMakeFiles/FMCWImplementation.dir/build

CMakeFiles/FMCWImplementation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FMCWImplementation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FMCWImplementation.dir/clean

CMakeFiles/FMCWImplementation.dir/depend:
	cd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build /home/david/Documents/RadarSecurityResearch/FMCW_radar_uhd/build/CMakeFiles/FMCWImplementation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FMCWImplementation.dir/depend

