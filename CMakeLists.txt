cmake_minimum_required(VERSION 3.15)
project(t3d2map CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -g -O0")

# Qt5 component for basic viewer
# find_package(CGAL REQUIRED COMPONENTS Qt5)
find_package(CGAL REQUIRED)

add_definitions(-DCGAL_USE_BASIC_VIEWER)

add_executable(${PROJECT_NAME} src/main.cpp)
# target_link_libraries(${PROJECT_NAME} CGAL::CGAL CGAL::CGAL_Qt5)
target_link_libraries(${PROJECT_NAME} CGAL::CGAL)
