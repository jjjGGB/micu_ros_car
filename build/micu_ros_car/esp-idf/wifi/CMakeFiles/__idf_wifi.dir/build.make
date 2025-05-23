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

# Include any dependencies generated for this target.
include esp-idf/wifi/CMakeFiles/__idf_wifi.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/wifi/CMakeFiles/__idf_wifi.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/wifi/CMakeFiles/__idf_wifi.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/wifi/CMakeFiles/__idf_wifi.dir/flags.make

esp-idf/wifi/CMakeFiles/__idf_wifi.dir/wifi.c.obj: esp-idf/wifi/CMakeFiles/__idf_wifi.dir/flags.make
esp-idf/wifi/CMakeFiles/__idf_wifi.dir/wifi.c.obj: ../../components/wifi/wifi.c
esp-idf/wifi/CMakeFiles/__idf_wifi.dir/wifi.c.obj: esp-idf/wifi/CMakeFiles/__idf_wifi.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/wifi/CMakeFiles/__idf_wifi.dir/wifi.c.obj"
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/wifi && /home/micu/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/wifi/CMakeFiles/__idf_wifi.dir/wifi.c.obj -MF CMakeFiles/__idf_wifi.dir/wifi.c.obj.d -o CMakeFiles/__idf_wifi.dir/wifi.c.obj -c /home/micu/esp/mycode/micu_ros_car/components/wifi/wifi.c

esp-idf/wifi/CMakeFiles/__idf_wifi.dir/wifi.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_wifi.dir/wifi.c.i"
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/wifi && /home/micu/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/micu/esp/mycode/micu_ros_car/components/wifi/wifi.c > CMakeFiles/__idf_wifi.dir/wifi.c.i

esp-idf/wifi/CMakeFiles/__idf_wifi.dir/wifi.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_wifi.dir/wifi.c.s"
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/wifi && /home/micu/.espressif/tools/xtensa-esp32s3-elf/esp-12.2.0_20230208/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/micu/esp/mycode/micu_ros_car/components/wifi/wifi.c -o CMakeFiles/__idf_wifi.dir/wifi.c.s

# Object files for target __idf_wifi
__idf_wifi_OBJECTS = \
"CMakeFiles/__idf_wifi.dir/wifi.c.obj"

# External object files for target __idf_wifi
__idf_wifi_EXTERNAL_OBJECTS =

esp-idf/wifi/libwifi.a: esp-idf/wifi/CMakeFiles/__idf_wifi.dir/wifi.c.obj
esp-idf/wifi/libwifi.a: esp-idf/wifi/CMakeFiles/__idf_wifi.dir/build.make
esp-idf/wifi/libwifi.a: esp-idf/wifi/CMakeFiles/__idf_wifi.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libwifi.a"
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/wifi && $(CMAKE_COMMAND) -P CMakeFiles/__idf_wifi.dir/cmake_clean_target.cmake
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/wifi && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_wifi.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/wifi/CMakeFiles/__idf_wifi.dir/build: esp-idf/wifi/libwifi.a
.PHONY : esp-idf/wifi/CMakeFiles/__idf_wifi.dir/build

esp-idf/wifi/CMakeFiles/__idf_wifi.dir/clean:
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/wifi && $(CMAKE_COMMAND) -P CMakeFiles/__idf_wifi.dir/cmake_clean.cmake
.PHONY : esp-idf/wifi/CMakeFiles/__idf_wifi.dir/clean

esp-idf/wifi/CMakeFiles/__idf_wifi.dir/depend:
	cd /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/micu/esp/mycode/micu_ros_car /home/micu/esp/mycode/micu_ros_car/components/wifi /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/wifi /home/micu/esp/mycode/micu_ros_car/build/micu_ros_car/esp-idf/wifi/CMakeFiles/__idf_wifi.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/wifi/CMakeFiles/__idf_wifi.dir/depend

