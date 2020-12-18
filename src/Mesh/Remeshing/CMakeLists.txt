cmake_minimum_required(VERSION 3.16)
project( Mesh_Remeshing )

set(CMAKE_CXX_STANDARD 14)

set(CGAL_DIR /home/yaoyu/Libraries/cgal)
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

set(EXE_MESH_REMESHING Mesh_Remeshing)

add_executable(${EXE_MESH_REMESHING} Remeshing.cpp)

target_compile_definitions(${EXE_MESH_REMESHING} PRIVATE TINYPLY_IMPLEMENTATION)
target_compile_definitions( ${EXE_MESH_REMESHING} PRIVATE CGAL_PMP_USE_CERES_SOLVER )

#target_link_libraries(${EXE_MESH_REMESHING} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../includes)
target_link_libraries(${EXE_MESH_REMESHING} PUBLIC ${CGAL_LIBRARIES} ceres
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        ${PCL_LIBRARIES})

# RemesingTest.
set(EXE_MESH_REMESHING_TEST Mesh_RemeshingTest)

add_executable(${EXE_MESH_REMESHING_TEST} RemeshingTest.cpp)

target_compile_definitions( ${EXE_MESH_REMESHING_TEST} PRIVATE TINYPLY_IMPLEMENTATION )
target_compile_definitions( ${EXE_MESH_REMESHING_TEST} PRIVATE CGAL_PMP_USE_CERES_SOLVER )

target_link_libraries(${EXE_MESH_REMESHING_TEST} PUBLIC ${CGAL_LIBRARIES} ceres
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        ${PCL_LIBRARIES})