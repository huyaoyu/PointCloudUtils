cmake_minimum_required(VERSION 3.16)
project( RemoveFaces )

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

set(EXE_REMOVE_FACES RemoveFaces)

add_executable(${EXE_REMOVE_FACES} RemoveFaces.cpp)

target_compile_definitions(${EXE_REMOVE_FACES} PRIVATE TINYPLY_IMPLEMENTATION)
target_compile_definitions( ${EXE_REMOVE_FACES} PRIVATE CGAL_PMP_USE_CERES_SOLVER )

#target_link_libraries(${EXE_REMOVE_FACES} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../includes)
target_link_libraries(${EXE_REMOVE_FACES} PUBLIC
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        ${CGAL_LIBRARIES} ceres)
