# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /opt/clion-2017.1.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /opt/clion-2017.1.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kelvimro/CLionProjects/TCCTerceira

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kelvimro/CLionProjects/TCCTerceira/cmake-build-debug

# Utility rule file for PLATFORMIO_BUILD.

# Include the progress variables for this target.
include CMakeFiles/PLATFORMIO_BUILD.dir/progress.make

CMakeFiles/PLATFORMIO_BUILD:
	cd /home/kelvimro/CLionProjects/TCCTerceira && /usr/local/bin/platformio -f -c clion run

PLATFORMIO_BUILD: CMakeFiles/PLATFORMIO_BUILD
PLATFORMIO_BUILD: CMakeFiles/PLATFORMIO_BUILD.dir/build.make

.PHONY : PLATFORMIO_BUILD

# Rule to build all files generated by this target.
CMakeFiles/PLATFORMIO_BUILD.dir/build: PLATFORMIO_BUILD

.PHONY : CMakeFiles/PLATFORMIO_BUILD.dir/build

CMakeFiles/PLATFORMIO_BUILD.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PLATFORMIO_BUILD.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PLATFORMIO_BUILD.dir/clean

CMakeFiles/PLATFORMIO_BUILD.dir/depend:
	cd /home/kelvimro/CLionProjects/TCCTerceira/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kelvimro/CLionProjects/TCCTerceira /home/kelvimro/CLionProjects/TCCTerceira /home/kelvimro/CLionProjects/TCCTerceira/cmake-build-debug /home/kelvimro/CLionProjects/TCCTerceira/cmake-build-debug /home/kelvimro/CLionProjects/TCCTerceira/cmake-build-debug/CMakeFiles/PLATFORMIO_BUILD.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PLATFORMIO_BUILD.dir/depend

