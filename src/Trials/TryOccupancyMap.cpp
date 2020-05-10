//
// Created by yaoyu on 5/7/20.
//

#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>

#include "OccupancyMap/OccupancyMap.hpp"
#include "PCCommon/IO.hpp"
#include "PCCommon/extraction.hpp"
#include "Profiling/SimpleTime.hpp"

int main(int argc, char** argv) {
    std::cout << "Hello, TryOccupancyMap! " << std::endl;

    assert(argc >= 3);

    std::cout << "Input point cloud is " << argv[1] << std::endl;
    std::cout << "Output octree file is " << argv[2] << std::endl;

    // Read a point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pInput =
            pcu::read_point_cloud<pcl::PointXYZ>(argv[1]);

    // Occupancy map.
    pcu::OccupancyMap ocMap;
    ocMap.initialize(0.05);

    // Origin of sensor.
    Eigen::Vector3f origin;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*pInput, centroid);
    origin << centroid(0), centroid(1), centroid(2);

    // Insert point cloud to the occupancy map.
    QUICK_TIME_START(teInsert)
    ocMap.insert_point_cloud<pcl::PointXYZ>(pInput, origin);
    QUICK_TIME_END(teInsert)
    std::cout << "Insert in " << teInsert << " ms. " << std::endl;

    // Write the occupancy map.
    ocMap.write(argv[2]);

    return 0;
}