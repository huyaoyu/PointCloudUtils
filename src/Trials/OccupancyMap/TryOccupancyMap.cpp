//
// Created by yaoyu on 5/13/20.
//

#define ENABLE_PROFILE 0

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "Exception/Common.hpp"
#include "OccupancyMap/OccupancyMap.hpp"
#include "Profiling/SimpleTime.hpp"

typedef std::uint8_t VT;

static const VT VISIBLE   = pcu::OCP_MAP_CAM_VISIBLE;
static const VT INVISIBLE = pcu::OCP_MAP_CAM_INVISIBLE;

int main(int argc, char** argv) {
    std::cout << "Hello, TryOccupancyMap! " << std::endl;
    std::cout << "Input octomap is " << argv[1] << std::endl;

    const std::string inOctoMap = argv[1];
    const std::string outDir    = argv[2];

    // Read the octomap.
    pcu::OccupancyMap ocMap;
    ocMap.read(inOctoMap);

    std::cout << "octomap has " << ocMap.get_octree().getNumLeafNodes() << " leaf nodes." << std::endl;
    ocMap.find_frontiers();
    {
        // Test use.
        std::string outFn = outDir + "/OccupancyMapVoxels.csv";
        ocMap.write_voxels_as_list(outFn);
    }

    return 0;
}