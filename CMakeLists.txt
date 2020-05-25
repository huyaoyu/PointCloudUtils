cmake_minimum_required(VERSION 3.15)
project(PointCloudUtils LANGUAGES CXX CUDA)

#if(COMMAND cmake_policy)
#    cmake_policy(SET CMP0003 NEW)
#    message(STATUS "CMP0003")
#endif(COMMAND cmake_policy)

set(CMAKE_CXX_STANDARD 14)

# ========== Libraries and includes. ==========

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

# OpenCV.
find_package(OpenCV REQUIRED PATHS $ENV{OCV4_ROOT}/lib/cmake/opencv4 NO_DEFAULT_PATH)
message(STATUS "OpenCV_VERSION = ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_INSTALL_PATH = ${OpenCV_INSTALL_PATH}")

FIND_LIBRARY( LIB_CNPY NAMES cnpy )
message(STATUS "LIB_CNPY=${LIB_CNPY}")

# Ceres Solver.
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# OctoMap.
find_package(octomap REQUIRED PATHS $ENV{OCTOMAP_ROOT}/share/octomap)
include_directories(${OCTOMAP_INCLUDE_DIRS})
message(STATUS "OCTOMAP_INCLUDE_DIRS=${OCTOMAP_INCLUDE_DIRS}")
message(STATUS "OCTOMAP_LIBRARIES=${OCTOMAP_LIBRARIES}")

# The yaml-cpp package.
find_package(yaml-cpp REQUIRED)
message(STATUS "yaml-cpp_FOUND=${yaml-cpp_FOUND}")
message(STATUS "YAML_CPP_INCLUDE_DIR=${yaml_cpp_INCLUDE_DIR}")
message(STATUS "YAML_CPP_LIBRARIES=${YAML_CPP_LIBRARIES}")

# CUDA.
find_package(CUDA REQUIRED)
message(STATUS "CUDA_INCLUDE_DIRS=${CUDA_INCLUDE_DIRS}")

include_directories(
        includes
        ${CUDA_INCLUDE_DIRS})

# ========== Local libraries. ==========

set(LIB_CV_COMMON cv_common)
add_library(${LIB_CV_COMMON} STATIC
        includes/CVCommon/IO.hpp src/CVCommon/IO.cpp )
target_include_directories(${LIB_CV_COMMON} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${LIB_CV_COMMON}
        Eigen3::Eigen
        ${OpenCV_LIBRARIES})

set(LIB_CUDA_OCP_BUILDER cuda_ocp_builder)
add_library(${LIB_CUDA_OCP_BUILDER} STATIC
        includes/OccupancyMap/CUDA/OccupancyMapBuilderRoutines.cu)
target_compile_features(${LIB_CUDA_OCP_BUILDER} PUBLIC cxx_std_11)
set_target_properties(${LIB_CUDA_OCP_BUILDER} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#find_package(Thrust REQUIRED CONFIG PATHS /usr/local/cuda-10.1/include/thrust/cmake)
#thrust_create_target(Thrust)

set(LIB_OCCUPANCY_MAP occupancy_map)
add_library(${LIB_OCCUPANCY_MAP} STATIC
        includes/OccupancyMap/OccupancyMap.hpp src/OccupancyMap/OccupancyMap.cpp )
set_target_properties(${LIB_OCCUPANCY_MAP} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_include_directories(${LIB_OCCUPANCY_MAP} PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_directories(${LIB_OCCUPANCY_MAP} PUBLIC ${OCTOMAP_LIBRARY_DIRS})
target_link_libraries(${LIB_OCCUPANCY_MAP}
        Eigen3::Eigen
        ${OCTOMAP_LIBRARIES}
        ${PCL_LIBRARIES}
        ${LIB_CUDA_OCP_BUILDER})

# ========== Tools. ==========

set(EXE_TOOL_DOWNSAMPLE DownSample)
add_executable(${EXE_TOOL_DOWNSAMPLE} src/Tools/DownSample.cpp)
target_link_libraries(${EXE_TOOL_DOWNSAMPLE} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_MOVE_CENTER MoveCenter)
add_executable(${EXE_TOOL_MOVE_CENTER} src/Tools/MoveCenter.cpp)
target_link_libraries(${EXE_TOOL_MOVE_CENTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_SMOOTH_FILTER SmoothFilter)
add_executable(${EXE_TOOL_SMOOTH_FILTER} src/Tools/SmoothFilter.cpp)
target_link_libraries(${EXE_TOOL_SMOOTH_FILTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_CROP_BBB CropByBBox)
add_executable(${EXE_TOOL_CROP_BBB} src/Tools/CropByOBBox.cpp)
target_link_libraries(${EXE_TOOL_CROP_BBB} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_NORMAL_ESTIMATION NormalEstimation)
add_executable(${EXE_TOOL_NORMAL_ESTIMATION} src/Tools/NormalEstimation.cpp)
target_link_libraries(${EXE_TOOL_NORMAL_ESTIMATION} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_CLUSTER_FILTER ClusterFilter)
add_executable(${EXE_TOOL_CLUSTER_FILTER} src/Tools/ClusterFilter.cpp)
target_link_libraries(${EXE_TOOL_CLUSTER_FILTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_SHADOW_CROP Tool_ShadowCrop)
add_executable(${EXE_TOOL_SHADOW_CROP}
        src/Tools/ShadowCrop.cpp)
target_link_libraries(${EXE_TOOL_SHADOW_CROP} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_ICP Tool_ICP)
add_executable(${EXE_TOOL_ICP}
        src/Tools/ICP.cpp)
target_link_libraries(${EXE_TOOL_ICP} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen ${PCL_LIBRARIES})

# ========== Trials. ==========

set(EXE_TRY_DISJOINT_SET Trial_TryDisjointSet)
add_executable(${EXE_TRY_DISJOINT_SET} src/Trials/TryDisjointSet.cpp)
target_link_libraries(${EXE_TRY_DISJOINT_SET} )

set(EXE_TRY_TRANSFORM_HELPERS Trial_TryTransformHelpers)
add_executable(${EXE_TRY_TRANSFORM_HELPERS} includes/Geometry/TransformHelpers.hpp src/Trials/TryTransformHelpers.cpp)
target_link_libraries(${EXE_TRY_TRANSFORM_HELPERS} Eigen3::Eigen)

set(EXE_CAM_GEOM_TRY_CSVIO Trial_CamGeomTryCSVIO)
add_executable(${EXE_CAM_GEOM_TRY_CSVIO}
        includes/CameraGeometry/IO.hpp
        includes/PCCommon/IO.hpp
        src/CameraGeometry/TryCSVIO.cpp)
target_link_libraries(${EXE_CAM_GEOM_TRY_CSVIO} Threads::Threads Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_CAM_GEOM_TRY_READ_MATRIX Trial_CamGeomTryReadCameraMatrix)
add_executable(${EXE_CAM_GEOM_TRY_READ_MATRIX}
        includes/DataInterfaces/Plain/Matrix.hpp
        src/CameraGeometry/TryReadCameraMatrix.cpp)
target_link_libraries(${EXE_CAM_GEOM_TRY_READ_MATRIX} Eigen3::Eigen)

set(EXE_CAMERA_GEOMETRY_TRY_EIGEN_QUATERNION Trial_CamGeomTryEigenQuaternion)
add_executable(${EXE_CAMERA_GEOMETRY_TRY_EIGEN_QUATERNION}
        src/CameraGeometry/TryEigenQuaternion.cpp)
target_link_libraries(${EXE_CAMERA_GEOMETRY_TRY_EIGEN_QUATERNION} Eigen3::Eigen)

set(EXE_YAML_TRY_YAML_CPP Trial_YAMLTryYamlCpp)
add_executable(${EXE_YAML_TRY_YAML_CPP}
        src/YAML/TryYamlCpp.cpp)
target_link_libraries(${EXE_YAML_TRY_YAML_CPP} Eigen3::Eigen yaml-cpp)

set(EXE_JSON_TRY_JSON Trial_JSONTryJSON)
add_executable(${EXE_JSON_TRY_JSON}
        src/JSON/TryJSON.cpp
        src/HoleBoundaryDetection/HoleBoundaryDetector.cpp)
target_link_libraries(${EXE_JSON_TRY_JSON} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TRIAL_TRY_EIGEN Trial_TryEigen)
add_executable(${EXE_TRIAL_TRY_EIGEN}
        src/Trials/TryEigen.cpp)
target_link_libraries(${EXE_TRIAL_TRY_EIGEN} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TRIAL_TRY_SMART_POINTER Trial_TrySmartPointer)
add_executable(${EXE_TRIAL_TRY_SMART_POINTER}
        src/Trials/TrySmartPointer.cpp)
#target_link_libraries(${EXE_TRIAL_TRY_SMART_POINTER})

set(EXE_TRIAL_TRY_CMATH Trial_TryCMath)
add_executable(${EXE_TRIAL_TRY_CMATH} src/Trials/TryCMath.cpp)

set(EXE_TRIAL_TRY_VECOTR Trial_TryVector)
add_executable(${EXE_TRIAL_TRY_VECOTR} src/Trials/TryVector.cpp)

set(EXE_TRIAL_TRY_OCTO_MAP Trial_TryOctoMap)
add_executable(${EXE_TRIAL_TRY_OCTO_MAP}
        src/Trials/OccupancyMap/TryOctoMap.cpp)
#target_link_directories(${EXE_TRIAL_TRY_OCTO_MAP} PRIVATE ${OCTOMAP_LIBRARY_DIRS})
set_target_properties(${EXE_TRIAL_TRY_OCTO_MAP} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(${EXE_TRIAL_TRY_OCTO_MAP}
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen
        ${LIB_OCCUPANCY_MAP} ${PCL_LIBRARIES}
        ${LIB_CUDA_OCP_BUILDER})

set(EXE_TRIAL_TRY_OCC_MAP Trial_TryOccMap)
add_executable(${EXE_TRIAL_TRY_OCC_MAP} src/Trials/OccupancyMap/TryOccupancyMap.cpp)
set_target_properties(${EXE_TRIAL_TRY_OCC_MAP} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(${EXE_TRIAL_TRY_OCC_MAP} Eigen3::Eigen
        ${LIB_OCCUPANCY_MAP})

set(EXE_TRIAL_TRY_OPENCV Trial_TryOpenCV)
add_executable(${EXE_TRIAL_TRY_OPENCV}
        src/Trials/TryOpenCV.cpp)
target_link_libraries(${EXE_TRIAL_TRY_OPENCV} ${OpenCV_LIBRARIES})

set(EXE_TRIAL_TRY_MOVE_SEMANTICS Trial_TryMoveSemantics)
add_executable(${EXE_TRIAL_TRY_MOVE_SEMANTICS}
        src/Trials/TryMoveSemantics.cpp)
#target_link_libraries(${EXE_TRIAL_TRY_MOVE_SEMANTICS})

# ========== Applications. ==========

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

set(EXE_BUILD_OCP_MAP BuildOccupancyMap)
add_executable(${EXE_BUILD_OCP_MAP}
        src/HoleBoundaryDetection/BuildOccupancyMap.cpp)
set_target_properties(${EXE_BUILD_OCP_MAP} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(${EXE_BUILD_OCP_MAP}
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen
        ${LIB_OCCUPANCY_MAP} ${PCL_LIBRARIES}
        ${LIB_CUDA_OCP_BUILDER} ${LIB_OCCUPANCY_MAP})

set(EXE_HOLE_BOUNDARY_PROJECTION HoleBoundaryProjection)
add_executable(${EXE_HOLE_BOUNDARY_PROJECTION}
        includes/PCCommon/extraction.hpp
        includes/PCCommon/IO.hpp
        src/HoleBoundaryDetection/HoleBoundaryProjection.cpp
        src/HoleBoundaryDetection/HoleBoundaryDetector.cpp)
target_link_libraries(${EXE_HOLE_BOUNDARY_PROJECTION}
        Threads::Threads
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES})

set(EXE_HOLE_BOUNDARY_SAVE_POLYGON_COOR HoleBoundarySavePolygonCoor)
add_executable(${EXE_HOLE_BOUNDARY_SAVE_POLYGON_COOR}
        src/HoleBoundaryDetection/SavePolygonCoordinates.cpp)
target_link_libraries(${EXE_HOLE_BOUNDARY_SAVE_POLYGON_COOR} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_HOLE_BOUNDARY_DENSITY_MAP HoleBoundaryDensityMap)
add_executable(${EXE_HOLE_BOUNDARY_DENSITY_MAP}
        src/HoleBoundaryDetection/DensityMap.cpp)
target_link_libraries(${EXE_HOLE_BOUNDARY_DENSITY_MAP}
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_HOLE_BOUNDARY_DEPTH_COMPLETION_DATA HoleDepthCompDataGen)
add_executable(${EXE_HOLE_BOUNDARY_DEPTH_COMPLETION_DATA}
        src/HoleBoundaryDetection/HoleDepthCompletionDataGeneration.cpp)
target_include_directories(${EXE_HOLE_BOUNDARY_DEPTH_COMPLETION_DATA} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${EXE_HOLE_BOUNDARY_DEPTH_COMPLETION_DATA}
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES} ${LIB_CV_COMMON}
        ${LIB_CNPY})

set(EXE_DEPTH_CPL_SIMPLE DepthCompletion_Simple)
add_executable(${EXE_DEPTH_CPL_SIMPLE} src/DepthCompletion/SimpleCameraPlane.cpp)
target_link_libraries(${EXE_DEPTH_CPL_SIMPLE}
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES} ${LIB_CV_COMMON}
        ${CERES_LIBRARIES}
        ${LIB_CNPY})

set(EXE_DEPTH_CPL_SIMPLE_BBOX DepthCompletion_SimpleBBoxPlane)
add_executable(${EXE_DEPTH_CPL_SIMPLE_BBOX} src/DepthCompletion/SimpleBBoxPlane.cpp)
target_link_libraries(${EXE_DEPTH_CPL_SIMPLE_BBOX}
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES} ${LIB_CV_COMMON}
        ${CERES_LIBRARIES}
        ${LIB_CNPY})