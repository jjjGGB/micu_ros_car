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
CMAKE_SOURCE_DIR = /home/micu/esp/mycode/micu_ros_car

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car

# Utility rule file for memory_ld.

# Include any custom commands dependencies for this target.
include esp-idf/esp_system/CMakeFiles/memory_ld.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/esp_system/CMakeFiles/memory_ld.dir/progress.make

esp-idf/esp_system/CMakeFiles/memory_ld: esp-idf/esp_system/ld/memory.ld

esp-idf/esp_system/ld/memory.ld: config/sdkconfig.h
esp-idf/esp_system/ld/memory.ld: /home/micu/esp/esp-idf/components/esp_system/ld/esp32s3/memory.ld.in
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating memory.ld linker script..."
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/esp_system && /home/micu/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc -C -P -x c -E -o /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/esp_system/ld/memory.ld -I /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/config -I /home/micu/esp/esp-idf/components/esp_system/ld /home/micu/esp/esp-idf/components/esp_system/ld/esp32s3/memory.ld.in

memory_ld: esp-idf/esp_system/CMakeFiles/memory_ld
memory_ld: esp-idf/esp_system/ld/memory.ld
memory_ld: esp-idf/esp_system/CMakeFiles/memory_ld.dir/build.make
.PHONY : memory_ld

# Rule to build all files generated by this target.
esp-idf/esp_system/CMakeFiles/memory_ld.dir/build: memory_ld
.PHONY : esp-idf/esp_system/CMakeFiles/memory_ld.dir/build

esp-idf/esp_system/CMakeFiles/memory_ld.dir/clean:
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/esp_system && $(CMAKE_COMMAND) -P CMakeFiles/memory_ld.dir/cmake_clean.cmake
.PHONY : esp-idf/esp_system/CMakeFiles/memory_ld.dir/clean

esp-idf/esp_system/CMakeFiles/memory_ld.dir/depend:
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/micu/esp/mycode/micu_ros_car /home/micu/esp/esp-idf/components/esp_system /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/esp_system /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/esp_system/CMakeFiles/memory_ld.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/esp_system/CMakeFiles/memory_ld.dir/depend

