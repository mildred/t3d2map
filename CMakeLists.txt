cmake_minimum_required(VERSION 2.8.6)
PROJECT(T3D2MAP)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# by default only QtCore and QtGui modules are enabled
# other modules must be enabled like this:
SET(QT_USE_QTTEST TRUE)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

#
# Depends on OpenGL
#

FIND_PACKAGE(OpenGL)

#
# Depends on VTK
#

find_package(VTK 5.10 REQUIRED)
include(${VTK_USE_FILE})

#
# Depends on CGAL
#

find_package(CGAL REQUIRED QUIET COMPONENTS Core )
include(${CGAL_USE_FILE})

#
# Sub-Projects
#

add_subdirectory(src)



