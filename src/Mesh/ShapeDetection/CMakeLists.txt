cmake_minimum_required(VERSION 3.16)
project( Mesh_ShapeDetection )

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

set(EXE_MESH_SHAPE_DETECTION Mesh_ShapeDetection)

add_executable(${EXE_MESH_SHAPE_DETECTION} ShapeDetection.cpp)

target_compile_definitions(${EXE_MESH_SHAPE_DETECTION} PRIVATE TINYPLY_IMPLEMENTATION)
target_compile_definitions( ${EXE_MESH_SHAPE_DETECTION} PRIVATE CGAL_PMP_USE_CERES_SOLVER )

#target_link_libraries(${EXE_MESH_SHAPE_DETECTION} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../includes)
target_link_libraries(${EXE_MESH_SHAPE_DETECTION} PUBLIC ${CGAL_LIBRARIES} CGAL::CGAL_Qt5 ceres
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        ${PCL_LIBRARIES})

# MovingPointsByPlaneDetection.
set(EXE_MESH_MOVING_POINTS_PLANE Mesh_MovingPointsByPlaneDetection)

add_executable(${EXE_MESH_MOVING_POINTS_PLANE} MovingPointsByPlaneDetection.cpp)

#target_compile_definitions(${EXE_MESH_MOVING_POINTS_PLANE} PRIVATE TINYPLY_IMPLEMENTATION)
#target_compile_definitions( ${EXE_MESH_MOVING_POINTS_PLANE} PRIVATE CGAL_PMP_USE_CERES_SOLVER )

#target_link_libraries(${EXE_MESH_MOVING_POINTS_PLANE} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../includes)
target_link_libraries(${EXE_MESH_MOVING_POINTS_PLANE} PUBLIC ${CGAL_LIBRARIES} CGAL::CGAL_Qt5
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS})