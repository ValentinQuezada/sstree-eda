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
CMAKE_SOURCE_DIR = /mnt/d/Documentos/UTEC-Programacion/EstructurasDatosAvanzados/lab4/code-new

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/d/Documentos/UTEC-Programacion/EstructurasDatosAvanzados/lab4/code-new/build

# Utility rule file for run_interface.

# Include any custom commands dependencies for this target.
include CMakeFiles/run_interface.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/run_interface.dir/progress.make

CMakeFiles/run_interface: ss_tree_interface
	./ss_tree_interface

run_interface: CMakeFiles/run_interface
run_interface: CMakeFiles/run_interface.dir/build.make
.PHONY : run_interface

# Rule to build all files generated by this target.
CMakeFiles/run_interface.dir/build: run_interface
.PHONY : CMakeFiles/run_interface.dir/build

CMakeFiles/run_interface.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_interface.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_interface.dir/clean

CMakeFiles/run_interface.dir/depend:
	cd /mnt/d/Documentos/UTEC-Programacion/EstructurasDatosAvanzados/lab4/code-new/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/d/Documentos/UTEC-Programacion/EstructurasDatosAvanzados/lab4/code-new /mnt/d/Documentos/UTEC-Programacion/EstructurasDatosAvanzados/lab4/code-new /mnt/d/Documentos/UTEC-Programacion/EstructurasDatosAvanzados/lab4/code-new/build /mnt/d/Documentos/UTEC-Programacion/EstructurasDatosAvanzados/lab4/code-new/build /mnt/d/Documentos/UTEC-Programacion/EstructurasDatosAvanzados/lab4/code-new/build/CMakeFiles/run_interface.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_interface.dir/depend

