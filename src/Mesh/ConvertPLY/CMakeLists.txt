cmake_minimum_required(VERSION 3.16)
project( ConvertPLY )

set(CMAKE_CXX_STANDARD 14)

set(CGAL_DIR /home/yaoyu/Libraries/cgal)
find_package(CGAL REQUIRED COMPONENTS Qt5 OPTIONAL_COMPONENTS Core )
message(STATUS "CGAL_LIBRARIES=${CGAL_LIBRARIES}")

if(CGAL_Qt5_FOUND)
    message(STATUS "CGAL_Qt5_FOUND")
    add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
else()
    message(STATUS "CGAL_Qt5 not found")
endif()

#dump_cmake_variables(CGAL)

# Boost and its components
find_package( Boost REQUIRED )
#dump_cmake_variables(Boost)

if ( NOT Boost_FOUND )

    message(STATUS "This project requires the Boost library, and will not be compiled.")

    return()

endif()

find_package(Ceres REQUIRED)
#dump_cmake_variables(Ceres)

set(EXE_CONVERT_PLY ConvertPLY)

add_executable(${EXE_CONVERT_PLY} ConvertPLY.cpp)

target_compile_definitions(${EXE_CONVERT_PLY} PRIVATE TINYPLY_IMPLEMENTATION)
target_compile_definitions( ${EXE_CONVERT_PLY} PRIVATE CGAL_PMP_USE_CERES_SOLVER )

#target_link_libraries(${EXE_CONVERT_PLY} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../includes)
target_link_libraries(${EXE_CONVERT_PLY} PUBLIC ${CGAL_LIBRARIES} ceres)
