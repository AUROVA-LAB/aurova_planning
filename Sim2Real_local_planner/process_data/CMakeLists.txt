cmake_minimum_required(VERSION 3.5.1)

project(planning VERSION 1.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(PCL REQUIRED COMPONENTS common filters)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(OpenMP REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
include(GNUInstallDirs)

IF (NOT CMAKE_BUILD_TYPE)
 #SET(CMAKE_BUILD_TYPE "DEBUG")
 SET(CMAKE_BUILD_TYPE "RELEASE") 
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

add_executable(calculate_safety_actions 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/obstacle_detection.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/safety_actions.cpp)

target_link_libraries(calculate_safety_actions 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    OpenMP::OpenMP_CXX
)
target_include_directories(calculate_safety_actions
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(calculate_safety_actions PRIVATE ${OpenMP_CXX_FLAGS})