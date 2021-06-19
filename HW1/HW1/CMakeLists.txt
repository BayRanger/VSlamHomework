PROJECT (sayhello)

cmake_minimum_required(VERSION 2.8)
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
add_library(hello SHARED src/hello.cpp)

add_executable(sayhello useHello.cpp)
target_link_libraries(sayhello hello)


INCLUDE_DIRECTORIES(include)
