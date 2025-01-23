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
CMAKE_SOURCE_DIR = /home/chris/loco-manipulation-cuda-DE

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chris/loco-manipulation-cuda-DE/build

# Include any dependencies generated for this target.
include CMakeFiles/cuda_solver.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cuda_solver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cuda_solver.dir/flags.make

CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.o: CMakeFiles/cuda_solver.dir/flags.make
CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.o: ../src/solver_center/solver_center.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chris/loco-manipulation-cuda-DE/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.o -c /home/chris/loco-manipulation-cuda-DE/src/solver_center/solver_center.cpp

CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chris/loco-manipulation-cuda-DE/src/solver_center/solver_center.cpp > CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.i

CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chris/loco-manipulation-cuda-DE/src/solver_center/solver_center.cpp -o CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.s

# Object files for target cuda_solver
cuda_solver_OBJECTS = \
"CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.o"

# External object files for target cuda_solver
cuda_solver_EXTERNAL_OBJECTS =

CMakeFiles/cuda_solver.dir/cmake_device_link.o: CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.o
CMakeFiles/cuda_solver.dir/cmake_device_link.o: CMakeFiles/cuda_solver.dir/build.make
CMakeFiles/cuda_solver.dir/cmake_device_link.o: libcuda_DE.a
CMakeFiles/cuda_solver.dir/cmake_device_link.o: /usr/local/cuda-11.1/lib64/libcublas.so
CMakeFiles/cuda_solver.dir/cmake_device_link.o: /usr/local/cuda-11.1/lib64/libcurand.so
CMakeFiles/cuda_solver.dir/cmake_device_link.o: /usr/local/lib/libyaml-cpp.a
CMakeFiles/cuda_solver.dir/cmake_device_link.o: /home/chris/.mujoco/mujoco-3.2.7/lib/libmujoco.so
CMakeFiles/cuda_solver.dir/cmake_device_link.o: CMakeFiles/cuda_solver.dir/dlink.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chris/loco-manipulation-cuda-DE/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CUDA device code CMakeFiles/cuda_solver.dir/cmake_device_link.o"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cuda_solver.dir/dlink.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cuda_solver.dir/build: CMakeFiles/cuda_solver.dir/cmake_device_link.o

.PHONY : CMakeFiles/cuda_solver.dir/build

# Object files for target cuda_solver
cuda_solver_OBJECTS = \
"CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.o"

# External object files for target cuda_solver
cuda_solver_EXTERNAL_OBJECTS =

../lib/DE_cuda_solver.so: CMakeFiles/cuda_solver.dir/src/solver_center/solver_center.cpp.o
../lib/DE_cuda_solver.so: CMakeFiles/cuda_solver.dir/build.make
../lib/DE_cuda_solver.so: libcuda_DE.a
../lib/DE_cuda_solver.so: /usr/local/cuda-11.1/lib64/libcublas.so
../lib/DE_cuda_solver.so: /usr/local/cuda-11.1/lib64/libcurand.so
../lib/DE_cuda_solver.so: /usr/local/lib/libyaml-cpp.a
../lib/DE_cuda_solver.so: /home/chris/.mujoco/mujoco-3.2.7/lib/libmujoco.so
../lib/DE_cuda_solver.so: CMakeFiles/cuda_solver.dir/cmake_device_link.o
../lib/DE_cuda_solver.so: CMakeFiles/cuda_solver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chris/loco-manipulation-cuda-DE/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module ../lib/DE_cuda_solver.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cuda_solver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cuda_solver.dir/build: ../lib/DE_cuda_solver.so

.PHONY : CMakeFiles/cuda_solver.dir/build

CMakeFiles/cuda_solver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cuda_solver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cuda_solver.dir/clean

CMakeFiles/cuda_solver.dir/depend:
	cd /home/chris/loco-manipulation-cuda-DE/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chris/loco-manipulation-cuda-DE /home/chris/loco-manipulation-cuda-DE /home/chris/loco-manipulation-cuda-DE/build /home/chris/loco-manipulation-cuda-DE/build /home/chris/loco-manipulation-cuda-DE/build/CMakeFiles/cuda_solver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cuda_solver.dir/depend

