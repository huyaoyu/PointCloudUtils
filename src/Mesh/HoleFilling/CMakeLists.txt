cmake_minimum_required(VERSION 3.16)
project( FillHoles )

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

set(EXE_FILL_HOLES Mesh_FillHoles)

add_executable(${EXE_FILL_HOLES} FillHoles.cpp)

target_compile_definitions(${EXE_FILL_HOLES} PRIVATE TINYPLY_IMPLEMENTATION)
target_compile_definitions( ${EXE_FILL_HOLES} PRIVATE CGAL_PMP_USE_CERES_SOLVER )

#target_link_libraries(${EXE_FILL_HOLES} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../includes)
target_link_libraries(${EXE_FILL_HOLES} PUBLIC ${CGAL_LIBRARIES} ceres
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        ${PCL_LIBRARIES})

# ========== FillHolesWithPlane. ==========

set(EXE_FILL_HOLES_WITH_PLANE Mesh_FillHolesWithPlane)

add_executable(${EXE_FILL_HOLES_WITH_PLANE} FillHolesWithPlane.cpp)

target_compile_definitions(${EXE_FILL_HOLES_WITH_PLANE} PRIVATE TINYPLY_IMPLEMENTATION)
target_compile_definitions( ${EXE_FILL_HOLES_WITH_PLANE} PRIVATE CGAL_PMP_USE_CERES_SOLVER )

#target_link_libraries(${EXE_FILL_HOLES_WITH_PLANE} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../includes)
target_link_libraries(${EXE_FILL_HOLES_WITH_PLANE} PUBLIC ${CGAL_LIBRARIES} ceres
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        ${PCL_LIBRARIES})