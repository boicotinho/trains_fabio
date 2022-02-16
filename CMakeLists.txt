cmake_minimum_required(VERSION 2.8.12)
project(trains)

#set(CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_CXX_STANDARD 11)
ADD_DEFINITIONS(-DBUILD_TYPE=\"${CMAKE_BUILD_TYPE}\")


add_compile_options(
    -Wall -Wextra -Werror -march=native -mtune=native # core-avx2
    -Wdeprecated -W -Wpointer-arith -Wno-uninitialized -Wno-unused
    -Wno-sign-compare -Wno-comment -Wno-unknown-pragmas -Wno-unused-parameter
    -Wno-invalid-offsetof -no-integrated-cpp -fno-strict-aliasing
    -Werror -Werror=return-type
    -fPIC
)

add_subdirectory(common)
add_subdirectory(generator)
add_subdirectory(visualizer)
add_subdirectory(solver)
