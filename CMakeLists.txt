# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "components" 
                         "components/lib"
                         "components/protocal"
                         "components/microros"
                         "../extra_components")

set(IDF_COMPONENT_MANAGER "0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(micu_ros_car)
