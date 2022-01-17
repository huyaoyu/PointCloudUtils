cmake_minimum_required(VERSION 3.16)
project( EnvelopSphere )

set(CMAKE_CXX_STANDARD 14)

set(CGAL_DIR $ENV{CGAL_DIR})
find_package(CGAL REQUIRED COMPONENTS Qt5 OPTIONAL_COMPONENTS Core )
message(STATUS "CGAL_LIBRARIES=${CGAL_LIBRARIES}")

if(CGAL_Qt5_FOUND)
    add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
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

set(EXE_ENVELOP_SPHERE EnvelopSphere)

add_executable(${EXE_ENVELOP_SPHERE} EnvelopSphere.cpp)

target_compile_definitions(${EXE_ENVELOP_SPHERE} PRIVATE TINYPLY_IMPLEMENTATION)
target_compile_definitions( ${EXE_ENVELOP_SPHERE} PRIVATE CGAL_PMP_USE_CERES_SOLVER )

target_link_libraries(${EXE_ENVELOP_SPHERE} PUBLIC ${CGAL_LIBRARIES} ceres)
