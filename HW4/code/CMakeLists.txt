cmake_minimum_required(VERSION 2.8)
project(image_process)
#fix cpp version
set (CMAKE_CXX_STANDARD 11)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package (Sophus REQUIRED)
find_package(OpenCV REQUIRED)
#include_directories(${Sophus_INCLUDE_DIRS})
#list(APPEND ALL_TARGET_LIBRARIES ${Sophus_LIBRARIES})
include_directories(
${PROJECT_SOURCE_DIR}
${Sophus_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}

)
add_executable(undistortImage undistort_image.cpp)

target_link_libraries(undistortImage
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Sophus_INCLUDE_DIRS}
${OpenCV_LIBS}  )

add_executable(disparity disparity.cpp)

target_link_libraries(disparity
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Sophus_INCLUDE_DIRS}
${OpenCV_LIBS}  )

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Sophus_INCLUDE_DIRS}
${OpenCV_LIBS}  )