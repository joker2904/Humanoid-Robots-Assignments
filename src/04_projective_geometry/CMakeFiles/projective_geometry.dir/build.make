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
CMAKE_SOURCE_DIR = /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry

# Include any dependencies generated for this target.
include CMakeFiles/projective_geometry.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/projective_geometry.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/projective_geometry.dir/flags.make

CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o: CMakeFiles/projective_geometry.dir/flags.make
CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o: src/ProjectiveGeometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o -c /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry/src/ProjectiveGeometry.cpp

CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry/src/ProjectiveGeometry.cpp > CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.i

CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry/src/ProjectiveGeometry.cpp -o CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.s

CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o.requires:

.PHONY : CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o.requires

CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o.provides: CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o.requires
	$(MAKE) -f CMakeFiles/projective_geometry.dir/build.make CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o.provides.build
.PHONY : CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o.provides

CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o.provides.build: CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o


# Object files for target projective_geometry
projective_geometry_OBJECTS = \
"CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o"

# External object files for target projective_geometry
projective_geometry_EXTERNAL_OBJECTS =

libprojective_geometry.a: CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o
libprojective_geometry.a: CMakeFiles/projective_geometry.dir/build.make
libprojective_geometry.a: CMakeFiles/projective_geometry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libprojective_geometry.a"
	$(CMAKE_COMMAND) -P CMakeFiles/projective_geometry.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/projective_geometry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/projective_geometry.dir/build: libprojective_geometry.a

.PHONY : CMakeFiles/projective_geometry.dir/build

CMakeFiles/projective_geometry.dir/requires: CMakeFiles/projective_geometry.dir/src/ProjectiveGeometry.cpp.o.requires

.PHONY : CMakeFiles/projective_geometry.dir/requires

CMakeFiles/projective_geometry.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/projective_geometry.dir/cmake_clean.cmake
.PHONY : CMakeFiles/projective_geometry.dir/clean

CMakeFiles/projective_geometry.dir/depend:
	cd /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry /home/manzil/Desktop/humanoid_robot/group-20/src/04_projective_geometry/CMakeFiles/projective_geometry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/projective_geometry.dir/depend

