cmake_minimum_required(VERSION 3.15)
project(PointCloudUtils)

set(CMAKE_CXX_STANDARD 14)

# Multi-thread.
find_package(Threads REQUIRED)
message(STATUS "Threads_FOUND=${Threads_FOUND}")

# Boost.
FIND_LIBRARY( LIB_BOOST_FILESYSTEM NAMES boost_filesystem )
FIND_LIBRARY( LIB_BOOST_SYSTEM NAMES boost_system )
FIND_LIBRARY( LIB_BOOST_PROGRAM_OPTIONS NAMES boost_program_options)
message(STATUS "LIB_BOOST_FILESYSTEM=${LIB_BOOST_FILESYSTEM}")
message(STATUS "LIB_BOOST_SYSTEM=${LIB_BOOST_SYSTEM}")
message(STATUS "LIB_BOOST_PROGRAM_OPTIONS=${LIB_BOOST_PROGRAM_OPTIONS}")

# Eigen 3.
FIND_PACKAGE(Eigen3 REQUIRED NO_MODULE)

# Find the PCL library.
find_package(PCL REQUIRED PATHS $ENV{PCL_ROOT} NO_DEFAULT_PATH)
message(STATUS "PCL_INCLUDE_DIRS=${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS=${PCL_LIBRARY_DIRS}")
add_definitions(${PCL_DEFINITIONS})

include_directories(includes)

set(EXE_DOWNSAMPLE DownSample)
add_executable(${EXE_DOWNSAMPLE} src/DownSample.cpp)
target_link_libraries(${EXE_DOWNSAMPLE} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_MOVE_CENTER MoveCenter)
add_executable(${EXE_MOVE_CENTER} src/MoveCenter.cpp)
target_link_libraries(${EXE_MOVE_CENTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_SMOOTH_FILTER SmoothFilter)
add_executable(${EXE_SMOOTH_FILTER} src/SmoothFilter.cpp)
target_link_libraries(${EXE_SMOOTH_FILTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_CROP_BBB CropByBBox)
add_executable(${EXE_CROP_BBB} src/CropByBBox.cpp)
target_link_libraries(${EXE_CROP_BBB} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_NORMAL_ESTIMATION NormalEstimation)
add_executable(${EXE_NORMAL_ESTIMATION} src/NormalEstimation.cpp)
target_link_libraries(${EXE_NORMAL_ESTIMATION} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_CLUSTER_FILTER ClusterFilter)
add_executable(${EXE_CLUSTER_FILTER} src/ClusterFilter.cpp)
target_link_libraries(${EXE_CLUSTER_FILTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TRY_DISJOINT_SET TryDisjointSet)
add_executable(${EXE_TRY_DISJOINT_SET} src/TryDisjointSet.cpp)
#target_link_libraries(${EXE_TRY_DISJOINT_SET} )

set(EXE_HOLE_BOUNDARY_DETECTION HoleBoundaryDetection)
add_executable(${EXE_HOLE_BOUNDARY_DETECTION}
        includes/Visualization/Color.hpp
        includes/PCCommon/extraction.hpp
        includes/PCCommon/IO.hpp
        includes/HoleBoundaryDetection/BoundaryCriterion.hpp
        src/HoleBoundaryDetection/ProximityGraph.cpp includes/HoleBoundaryDetection/ProximityGraph.hpp
        src/HoleBoundaryDetection/HoleBoundaryDetector.cpp includes/HoleBoundaryDetection/HoleBoundaryDetector.hpp
        src/HoleBoundaryDetection/HoleBoundaryDetection.cpp)
target_link_libraries(${EXE_HOLE_BOUNDARY_DETECTION} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TRY_TRANSFORM_HELPERS TryTransformHelpers)
add_executable(${EXE_TRY_TRANSFORM_HELPERS} includes/Geometry/TransformHelpers.hpp src/TryTransformHelpers.cpp)
target_link_libraries(${EXE_TRY_TRANSFORM_HELPERS} Eigen3::Eigen)

set(EXE_CAM_GEOM_TRY_CSVIO CamGeom_TryCSVIO)
add_executable(${EXE_CAM_GEOM_TRY_CSVIO}
        includes/CameraGeometry/IO.hpp
        includes/PCCommon/IO.hpp
        src/CameraGeometry/TryCSVIO.cpp)
target_link_libraries(${EXE_CAM_GEOM_TRY_CSVIO} Threads::Threads Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_CAM_GEOM_TRY_READ_MATRIX CamGeom_TryReadCameraMatrix)
add_executable(${EXE_CAM_GEOM_TRY_READ_MATRIX}
        includes/DataInterfaces/Plain/Matrix.hpp
        src/CameraGeometry/TryReadCameraMatrix.cpp)
target_link_libraries(${EXE_CAM_GEOM_TRY_READ_MATRIX} Eigen3::Eigen)