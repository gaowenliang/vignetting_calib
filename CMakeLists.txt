cmake_minimum_required(VERSION 2.8.3)
project(vignetting_model)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC -fopenmp")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    camera_model
    code_utils
    )


find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
#set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES vignetting_model
#    CATKIN_DEPENDS roscpp std_msgs
#    DEPENDS system_lib
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
     )

include_directories("include")

add_library(vignetting_model STATIC
    src/vignetting/vignettingcalib.cpp
    src/vignetting/vignettingtable.cpp
    src/vignetting/vignetting.cpp
    )
target_link_libraries(vignetting_model
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS} )

add_executable(vignetting_calib
    src/vignetting/vignettingcalib.cpp
    src/vignetting/vignettingtable.cpp
    src/vignetting/vignetting.cpp
    src/vignetting_calib.cc)

target_link_libraries(vignetting_calib dw
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES} )

add_executable(vignetting_test
    src/vignetting/vignettingcalib.cpp
    src/vignetting/vignettingtable.cpp
    src/vignetting/vignetting.cpp
    src/vignetting_test.cc)

target_link_libraries(vignetting_test dw
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES} )

