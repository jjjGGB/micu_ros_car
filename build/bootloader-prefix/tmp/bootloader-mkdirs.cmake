# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espresssif/container/v5.1.4/esp-idf/components/bootloader/subproject"
  "D:/Espresssif/mycode/micu_ros_car/build/bootloader"
  "D:/Espresssif/mycode/micu_ros_car/build/bootloader-prefix"
  "D:/Espresssif/mycode/micu_ros_car/build/bootloader-prefix/tmp"
  "D:/Espresssif/mycode/micu_ros_car/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Espresssif/mycode/micu_ros_car/build/bootloader-prefix/src"
  "D:/Espresssif/mycode/micu_ros_car/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Espresssif/mycode/micu_ros_car/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Espresssif/mycode/micu_ros_car/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
