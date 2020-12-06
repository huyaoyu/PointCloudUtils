# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.

cmake_minimum_required(VERSION 3.16)
project( CGAL5 )

#function(dump_cmake_variables)
#    get_cmake_property(_variableNames VARIABLES)
#    list (SORT _variableNames)
#    foreach (_variableName ${_variableNames})
#        if (ARGV0)
#            unset(MATCHED)
#            string(REGEX MATCH ${ARGV0} MATCHED ${_variableName})
#            if (NOT MATCHED)
#                continue()
#            endif()
#        endif()
#        message(STATUS "${_variableName}=${${_variableName}}")
#    endforeach()
#endfunction()

set(CMAKE_CXX_STANDARD 14)

set(CGAL_DIR /home/yaoyu/Libraries/cgal)
find_package(CGAL REQUIRED COMPONENTS Qt5 OPTIONAL_COMPONENTS Core )
message(STATUS "CGAL_LIBRARIES=${CGAL_LIBRARIES}")

if(CGAL_Qt5_FOUND)
    add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
endif()

#dump_cmake_variables(CGAL)

include_directories(includes)

add_subdirectory(ConvertPLY)
add_subdirectory(EnvelopSphere)
add_subdirectory(HoleFilling)
add_subdirectory(RemoveFaces)
add_subdirectory(Reconstruction)
add_subdirectory(Remeshing)
add_subdirectory(ShapeDetection)
add_subdirectory(Simplification)