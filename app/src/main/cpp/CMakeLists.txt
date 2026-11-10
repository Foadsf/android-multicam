cmake_minimum_required(VERSION 3.22.1)
project("multicam_native" LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(multicam_native SHARED native-lib.cpp engine.hpp)
target_compile_options(multicam_native PRIVATE -O3 -fno-math-errno -Wall -Wextra)

find_library(log-lib log)
find_library(android-lib android)
find_library(jnigraphics-lib jnigraphics)
target_link_libraries(multicam_native ${log-lib} ${android-lib} ${jnigraphics-lib})
