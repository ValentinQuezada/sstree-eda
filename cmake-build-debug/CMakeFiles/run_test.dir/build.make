# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.20

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "D:\Descargas\CLion 2021.2.1\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "D:\Descargas\CLion 2021.2.1\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\Documentos\UTEC-Programacion\EstructurasDatosAvanzados\lab4\code-new

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\Documentos\UTEC-Programacion\EstructurasDatosAvanzados\lab4\code-new\cmake-build-debug

# Utility rule file for run_test.

# Include any custom commands dependencies for this target.
include CMakeFiles/run_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/run_test.dir/progress.make

CMakeFiles/run_test: ss_tree_test.exe
	.\ss_tree_test.exe

run_test: CMakeFiles/run_test
run_test: CMakeFiles/run_test.dir/build.make
.PHONY : run_test

# Rule to build all files generated by this target.
CMakeFiles/run_test.dir/build: run_test
.PHONY : CMakeFiles/run_test.dir/build

CMakeFiles/run_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\run_test.dir\cmake_clean.cmake
.PHONY : CMakeFiles/run_test.dir/clean

CMakeFiles/run_test.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Documentos\UTEC-Programacion\EstructurasDatosAvanzados\lab4\code-new D:\Documentos\UTEC-Programacion\EstructurasDatosAvanzados\lab4\code-new D:\Documentos\UTEC-Programacion\EstructurasDatosAvanzados\lab4\code-new\cmake-build-debug D:\Documentos\UTEC-Programacion\EstructurasDatosAvanzados\lab4\code-new\cmake-build-debug D:\Documentos\UTEC-Programacion\EstructurasDatosAvanzados\lab4\code-new\cmake-build-debug\CMakeFiles\run_test.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_test.dir/depend

