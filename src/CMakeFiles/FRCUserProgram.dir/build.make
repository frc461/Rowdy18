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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/c/Users/WBI/workspace/Rowdy18

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/WBI/workspace/Rowdy18

# Include any dependencies generated for this target.
include src/CMakeFiles/FRCUserProgram.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/FRCUserProgram.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/FRCUserProgram.dir/flags.make

src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o: src/CMakeFiles/FRCUserProgram.dir/flags.make
src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o: src/Robot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /mnt/c/Users/WBI/workspace/Rowdy18/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o"
	cd /mnt/c/Users/WBI/workspace/Rowdy18/src && /usr/bin/arm-frc-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/FRCUserProgram.dir/Robot.cpp.o -c /mnt/c/Users/WBI/workspace/Rowdy18/src/Robot.cpp

src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FRCUserProgram.dir/Robot.cpp.i"
	cd /mnt/c/Users/WBI/workspace/Rowdy18/src && /usr/bin/arm-frc-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /mnt/c/Users/WBI/workspace/Rowdy18/src/Robot.cpp > CMakeFiles/FRCUserProgram.dir/Robot.cpp.i

src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FRCUserProgram.dir/Robot.cpp.s"
	cd /mnt/c/Users/WBI/workspace/Rowdy18/src && /usr/bin/arm-frc-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /mnt/c/Users/WBI/workspace/Rowdy18/src/Robot.cpp -o CMakeFiles/FRCUserProgram.dir/Robot.cpp.s

src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o.requires:
.PHONY : src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o.requires

src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o.provides: src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/FRCUserProgram.dir/build.make src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o.provides.build
.PHONY : src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o.provides

src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o.provides.build: src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o

src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o: src/CMakeFiles/FRCUserProgram.dir/flags.make
src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o: src/RateEncoder.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /mnt/c/Users/WBI/workspace/Rowdy18/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o"
	cd /mnt/c/Users/WBI/workspace/Rowdy18/src && /usr/bin/arm-frc-linux-gnueabi-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o -c /mnt/c/Users/WBI/workspace/Rowdy18/src/RateEncoder.cpp

src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.i"
	cd /mnt/c/Users/WBI/workspace/Rowdy18/src && /usr/bin/arm-frc-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /mnt/c/Users/WBI/workspace/Rowdy18/src/RateEncoder.cpp > CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.i

src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.s"
	cd /mnt/c/Users/WBI/workspace/Rowdy18/src && /usr/bin/arm-frc-linux-gnueabi-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /mnt/c/Users/WBI/workspace/Rowdy18/src/RateEncoder.cpp -o CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.s

src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o.requires:
.PHONY : src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o.requires

src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o.provides: src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/FRCUserProgram.dir/build.make src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o.provides.build
.PHONY : src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o.provides

src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o.provides.build: src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o

# Object files for target FRCUserProgram
FRCUserProgram_OBJECTS = \
"CMakeFiles/FRCUserProgram.dir/Robot.cpp.o" \
"CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o"

# External object files for target FRCUserProgram
FRCUserProgram_EXTERNAL_OBJECTS =

src/Debug/FRCUserProgram: src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o
src/Debug/FRCUserProgram: src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o
src/Debug/FRCUserProgram: src/CMakeFiles/FRCUserProgram.dir/build.make
src/Debug/FRCUserProgram: src/CMakeFiles/FRCUserProgram.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Debug/FRCUserProgram"
	cd /mnt/c/Users/WBI/workspace/Rowdy18/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FRCUserProgram.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/FRCUserProgram.dir/build: src/Debug/FRCUserProgram
.PHONY : src/CMakeFiles/FRCUserProgram.dir/build

src/CMakeFiles/FRCUserProgram.dir/requires: src/CMakeFiles/FRCUserProgram.dir/Robot.cpp.o.requires
src/CMakeFiles/FRCUserProgram.dir/requires: src/CMakeFiles/FRCUserProgram.dir/RateEncoder.cpp.o.requires
.PHONY : src/CMakeFiles/FRCUserProgram.dir/requires

src/CMakeFiles/FRCUserProgram.dir/clean:
	cd /mnt/c/Users/WBI/workspace/Rowdy18/src && $(CMAKE_COMMAND) -P CMakeFiles/FRCUserProgram.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/FRCUserProgram.dir/clean

src/CMakeFiles/FRCUserProgram.dir/depend:
	cd /mnt/c/Users/WBI/workspace/Rowdy18 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/WBI/workspace/Rowdy18 /mnt/c/Users/WBI/workspace/Rowdy18/src /mnt/c/Users/WBI/workspace/Rowdy18 /mnt/c/Users/WBI/workspace/Rowdy18/src /mnt/c/Users/WBI/workspace/Rowdy18/src/CMakeFiles/FRCUserProgram.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/FRCUserProgram.dir/depend
