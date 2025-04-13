cmake_minimum_required(VERSION 3.5)

# Add the root of this git repo to the component search path.
set(EXTRA_COMPONENT_DIRS "../../")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(max_cpp)
