# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/joan/ClionProjects/ApiClient

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/joan/ClionProjects/ApiClient/build

# Include any dependencies generated for this target.
include CMakeFiles/ApiClient.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ApiClient.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ApiClient.dir/flags.make

CMakeFiles/ApiClient.dir/src/Core.cpp.o: CMakeFiles/ApiClient.dir/flags.make
CMakeFiles/ApiClient.dir/src/Core.cpp.o: ../src/Core.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joan/ClionProjects/ApiClient/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ApiClient.dir/src/Core.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ApiClient.dir/src/Core.cpp.o -c /home/joan/ClionProjects/ApiClient/src/Core.cpp

CMakeFiles/ApiClient.dir/src/Core.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ApiClient.dir/src/Core.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joan/ClionProjects/ApiClient/src/Core.cpp > CMakeFiles/ApiClient.dir/src/Core.cpp.i

CMakeFiles/ApiClient.dir/src/Core.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ApiClient.dir/src/Core.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joan/ClionProjects/ApiClient/src/Core.cpp -o CMakeFiles/ApiClient.dir/src/Core.cpp.s

CMakeFiles/ApiClient.dir/src/Core.cpp.o.requires:

.PHONY : CMakeFiles/ApiClient.dir/src/Core.cpp.o.requires

CMakeFiles/ApiClient.dir/src/Core.cpp.o.provides: CMakeFiles/ApiClient.dir/src/Core.cpp.o.requires
	$(MAKE) -f CMakeFiles/ApiClient.dir/build.make CMakeFiles/ApiClient.dir/src/Core.cpp.o.provides.build
.PHONY : CMakeFiles/ApiClient.dir/src/Core.cpp.o.provides

CMakeFiles/ApiClient.dir/src/Core.cpp.o.provides.build: CMakeFiles/ApiClient.dir/src/Core.cpp.o


CMakeFiles/ApiClient.dir/src/main.cpp.o: CMakeFiles/ApiClient.dir/flags.make
CMakeFiles/ApiClient.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joan/ClionProjects/ApiClient/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ApiClient.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ApiClient.dir/src/main.cpp.o -c /home/joan/ClionProjects/ApiClient/src/main.cpp

CMakeFiles/ApiClient.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ApiClient.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joan/ClionProjects/ApiClient/src/main.cpp > CMakeFiles/ApiClient.dir/src/main.cpp.i

CMakeFiles/ApiClient.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ApiClient.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joan/ClionProjects/ApiClient/src/main.cpp -o CMakeFiles/ApiClient.dir/src/main.cpp.s

CMakeFiles/ApiClient.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/ApiClient.dir/src/main.cpp.o.requires

CMakeFiles/ApiClient.dir/src/main.cpp.o.provides: CMakeFiles/ApiClient.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/ApiClient.dir/build.make CMakeFiles/ApiClient.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/ApiClient.dir/src/main.cpp.o.provides

CMakeFiles/ApiClient.dir/src/main.cpp.o.provides.build: CMakeFiles/ApiClient.dir/src/main.cpp.o


# Object files for target ApiClient
ApiClient_OBJECTS = \
"CMakeFiles/ApiClient.dir/src/Core.cpp.o" \
"CMakeFiles/ApiClient.dir/src/main.cpp.o"

# External object files for target ApiClient
ApiClient_EXTERNAL_OBJECTS =

ApiClient: CMakeFiles/ApiClient.dir/src/Core.cpp.o
ApiClient: CMakeFiles/ApiClient.dir/src/main.cpp.o
ApiClient: CMakeFiles/ApiClient.dir/build.make
ApiClient: ext/ApiCodec/libapicodec.so.0.0.1
ApiClient: CMakeFiles/ApiClient.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/joan/ClionProjects/ApiClient/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ApiClient"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ApiClient.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ApiClient.dir/build: ApiClient

.PHONY : CMakeFiles/ApiClient.dir/build

CMakeFiles/ApiClient.dir/requires: CMakeFiles/ApiClient.dir/src/Core.cpp.o.requires
CMakeFiles/ApiClient.dir/requires: CMakeFiles/ApiClient.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/ApiClient.dir/requires

CMakeFiles/ApiClient.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ApiClient.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ApiClient.dir/clean

CMakeFiles/ApiClient.dir/depend:
	cd /home/joan/ClionProjects/ApiClient/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/joan/ClionProjects/ApiClient /home/joan/ClionProjects/ApiClient /home/joan/ClionProjects/ApiClient/build /home/joan/ClionProjects/ApiClient/build /home/joan/ClionProjects/ApiClient/build/CMakeFiles/ApiClient.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ApiClient.dir/depend

