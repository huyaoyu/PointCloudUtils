cmake_minimum_required(VERSION 3.15)
project(PointCloudTools LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 14)

set(RELATIVE_ROOT ../../)

set(EXE_TOOL_DOWNSAMPLE Tool_DownSample)
add_executable(${EXE_TOOL_DOWNSAMPLE} DownSample.cpp)
target_link_libraries(${EXE_TOOL_DOWNSAMPLE} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_MOVE_CENTER MoveCenter)
add_executable(${EXE_TOOL_MOVE_CENTER} MoveCenter.cpp)
target_link_libraries(${EXE_TOOL_MOVE_CENTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_SMOOTH_FILTER SmoothFilter)
add_executable(${EXE_TOOL_SMOOTH_FILTER} SmoothFilter.cpp)
target_link_libraries(${EXE_TOOL_SMOOTH_FILTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_CROP_BBB CropByBBox)
add_executable(${EXE_TOOL_CROP_BBB} CropByOBBox.cpp)
target_link_libraries(${EXE_TOOL_CROP_BBB} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_NORMAL_ESTIMATION NormalEstimation)
add_executable(${EXE_TOOL_NORMAL_ESTIMATION} NormalEstimation.cpp)
target_link_libraries(${EXE_TOOL_NORMAL_ESTIMATION} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_CLUSTER_FILTER ClusterFilter)
add_executable(${EXE_TOOL_CLUSTER_FILTER} ClusterFilter.cpp)
target_link_libraries(${EXE_TOOL_CLUSTER_FILTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_COLOR_FILTER Tool_ColorFilter)
add_executable(${EXE_TOOL_COLOR_FILTER} ColorFilter.cpp)
target_link_libraries(${EXE_TOOL_COLOR_FILTER} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_SHADOW_CROP Tool_ShadowCrop)
add_executable(${EXE_TOOL_SHADOW_CROP}
        ShadowCrop.cpp)
target_link_libraries(${EXE_TOOL_SHADOW_CROP} Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_ICP Tool_ICP)
add_executable(${EXE_TOOL_ICP}
        ICP.cpp)
target_link_libraries(${EXE_TOOL_ICP} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_SEARCH_MERGE Tool_SearchMergePointClouds)
add_executable(${EXE_TOOL_SEARCH_MERGE}
        SearchMergePointClouds.cpp)
target_link_libraries(${EXE_TOOL_SEARCH_MERGE} ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen ${PCL_LIBRARIES})

set(EXE_TOOL_MERGE Tool_Merge)
add_executable(${EXE_TOOL_MERGE} Merge.cpp)
target_link_libraries(${EXE_TOOL_MERGE}
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen
        ${PCL_LIBRARIES})

set(EXE_TOOL_FLIP_NORMAL Tool_FlipNormal)
add_executable( ${EXE_TOOL_FLIP_NORMAL} FlipNormal.cpp )
target_link_libraries( ${EXE_TOOL_FLIP_NORMAL}
        ${LIB_BOOST_SYSTEM} ${LIB_BOOST_FILESYSTEM} ${LIB_BOOST_PROGRAM_OPTIONS}
        Eigen3::Eigen
        ${PCL_LIBRARIES})