# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/ubuntu/myThesis

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/myThesis/build

# Include any dependencies generated for this target.
include CMakeFiles/compVision.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/compVision.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/compVision.dir/flags.make

CMakeFiles/compVision.dir/Comp_Vision.cpp.o: CMakeFiles/compVision.dir/flags.make
CMakeFiles/compVision.dir/Comp_Vision.cpp.o: Comp_Vision.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/myThesis/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/compVision.dir/Comp_Vision.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/compVision.dir/Comp_Vision.cpp.o -c /home/ubuntu/myThesis/build/Comp_Vision.cpp

CMakeFiles/compVision.dir/Comp_Vision.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/compVision.dir/Comp_Vision.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/myThesis/build/Comp_Vision.cpp > CMakeFiles/compVision.dir/Comp_Vision.cpp.i

CMakeFiles/compVision.dir/Comp_Vision.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/compVision.dir/Comp_Vision.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/myThesis/build/Comp_Vision.cpp -o CMakeFiles/compVision.dir/Comp_Vision.cpp.s

# Object files for target compVision
compVision_OBJECTS = \
"CMakeFiles/compVision.dir/Comp_Vision.cpp.o"

# External object files for target compVision
compVision_EXTERNAL_OBJECTS =

compVision: CMakeFiles/compVision.dir/Comp_Vision.cpp.o
compVision: CMakeFiles/compVision.dir/build.make
compVision: CMakeFiles/compVision.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/myThesis/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable compVision"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/compVision.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/compVision.dir/build: compVision

.PHONY : CMakeFiles/compVision.dir/build

CMakeFiles/compVision.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/compVision.dir/cmake_clean.cmake
.PHONY : CMakeFiles/compVision.dir/clean

CMakeFiles/compVision.dir/depend:
	cd /home/ubuntu/myThesis/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/myThesis /home/ubuntu/myThesis /home/ubuntu/myThesis/build /home/ubuntu/myThesis/build /home/ubuntu/myThesis/build/CMakeFiles/compVision.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/compVision.dir/depend

