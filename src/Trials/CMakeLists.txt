cmake_minimum_required(VERSION 3.15)
project(Trials LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 14)

set(RELATIVE_ROOT ../../)

set(EXE_TRY_DISJOINT_SET Trial_TryDisjointSet)
add_executable(${EXE_TRY_DISJOINT_SET} TryDisjointSet.cpp)
target_link_libraries(${EXE_TRY_DISJOINT_SET} )

set(EXE_TRY_TRANSFORM_HELPERS Trial_TryTransformHelpers)
add_executable(${EXE_TRY_TRANSFORM_HELPERS}
        ${RELATIVE_ROOT}includes/Geometry/TransformHelpers.hpp
        TryTransformHelpers.cpp)
target_link_libraries(${EXE_TRY_TRANSFORM_HELPERS} Eigen3::Eigen)

set(EXE_CAM_GEOM_TRY_CSVIO Trial_CamGeomTryCSVIO)
add_executable(${EXE_CAM_GEOM_TRY_CSVIO}
        ${RELATIVE_ROOT}includes/CameraGeometry/IO.hpp
        ${RELATIVE_ROOT}includes/PCCommon/IO.hpp
        CameraGeometry/TryCSVIO.cpp)
target_link_libraries(${EXE_CAM_GEOM_TRY_CSVIO} Threads::Threads Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_CAM_GEOM_TRY_READ_MATRIX Trial_CamGeomTryReadCameraMatrix)
add_executable(${EXE_CAM_GEOM_TRY_READ_MATRIX}
        ${RELATIVE_ROOT}includes/DataInterfaces/Plain/Matrix.hpp
        CameraGeometry/TryReadCameraMatrix.cpp)
target_link_libraries(${EXE_CAM_GEOM_TRY_READ_MATRIX} Eigen3::Eigen)

set(EXE_CAMERA_GEOMETRY_TRY_EIGEN_QUATERNION Trial_CamGeomTryEigenQuaternion)
add_executable(${EXE_CAMERA_GEOMETRY_TRY_EIGEN_QUATERNION}
        CameraGeometry/TryEigenQuaternion.cpp)
target_link_libraries(${EXE_CAMERA_GEOMETRY_TRY_EIGEN_QUATERNION} Eigen3::Eigen)

set(EXE_YAML_TRY_YAML_CPP Trial_YAMLTryYamlCpp)
add_executable(${EXE_YAML_TRY_YAML_CPP}
        YAML/TryYamlCpp.cpp)
target_link_libraries(${EXE_YAML_TRY_YAML_CPP} Eigen3::Eigen yaml-cpp)

set(EXE_JSON_TRY_JSON Trial_JSONTryJSON)
add_executable(${EXE_JSON_TRY_JSON}
        JSON/TryJSON.cpp ${RELATIVE_ROOT}/src/HoleBoundaryDetection/HoleBoundaryDetector.cpp)
target_link_libraries(${EXE_JSON_TRY_JSON} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TRIAL_TRY_EIGEN Trial_TryEigen)
add_executable(${EXE_TRIAL_TRY_EIGEN}
        TryEigen.cpp)
target_link_libraries(${EXE_TRIAL_TRY_EIGEN} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TRIAL_TRY_SMART_POINTER Trial_TrySmartPointer)
add_executable(${EXE_TRIAL_TRY_SMART_POINTER}
        TrySmartPointer.cpp)
#target_link_libraries(${EXE_TRIAL_TRY_SMART_POINTER})

set(EXE_TRIAL_TRY_CMATH Trial_TryCMath)
add_executable(${EXE_TRIAL_TRY_CMATH} TryCMath.cpp)

set(EXE_TRIAL_TRY_VECOTR Trial_TryVector)
add_executable(${EXE_TRIAL_TRY_VECOTR} TryVector.cpp)

set(EXE_TRIAL_TRY_OCTO_MAP Trial_TryOctoMap)
add_executable(${EXE_TRIAL_TRY_OCTO_MAP}
        OccupancyMap/TryOctoMap.cpp)
#target_link_directories(${EXE_TRIAL_TRY_OCTO_MAP} PRIVATE ${OCTOMAP_LIBRARY_DIRS})
set_target_properties(${EXE_TRIAL_TRY_OCTO_MAP} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(${EXE_TRIAL_TRY_OCTO_MAP}
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen
        ${LIB_OCCUPANCY_MAP} ${PCL_LIBRARIES}
        ${LIB_CUDA_OCP_BUILDER})

set(EXE_TRIAL_TRY_OCC_MAP Trial_TryOccMap)
add_executable(${EXE_TRIAL_TRY_OCC_MAP} OccupancyMap/TryOccupancyMap.cpp)
set_target_properties(${EXE_TRIAL_TRY_OCC_MAP} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(${EXE_TRIAL_TRY_OCC_MAP} Eigen3::Eigen
        ${LIB_OCCUPANCY_MAP})

set(EXE_TRIAL_TRY_OPENCV Trial_TryOpenCV)
add_executable(${EXE_TRIAL_TRY_OPENCV}
        TryOpenCV.cpp)
target_link_libraries(${EXE_TRIAL_TRY_OPENCV} ${OpenCV_LIBRARIES})

set(EXE_TRIAL_TRY_MOVE_SEMANTICS Trial_TryMoveSemantics)
add_executable(${EXE_TRIAL_TRY_MOVE_SEMANTICS}
        TryMoveSemantics.cpp)
#target_link_libraries(${EXE_TRIAL_TRY_MOVE_SEMANTICS})

set(EXE_TRIAL_TRY_ARGUMENT_PARSER Trial_TryArgumentParser)
add_executable(${EXE_TRIAL_TRY_ARGUMENT_PARSER} TryArgumentParser.cpp)
target_link_libraries(${EXE_TRIAL_TRY_ARGUMENT_PARSER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS})