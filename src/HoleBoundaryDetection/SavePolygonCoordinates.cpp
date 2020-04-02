//
// Created by yaoyu on 4/2/20.
//

#include <fstream>
#include <iostream>
#include <vector>

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

#include "PCCommon/extraction.hpp"
#include "PCCommon/IO.hpp"
#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"

using json = nlohmann::json;

int main( int argc, char* argv[] ) {
    std::cout << "Hello, SavePolygonCoordinats! " << std::endl;

    pcl::PointCloud<pcl::PointNormal>::Ptr pInput =
            pcu::read_point_cloud<pcl::PointNormal>( argv[1] );

    std::ifstream ifs(argv[2]);
    json jExt;
    ifs >> jExt;

    std::vector<int> indices = jExt["hpp"][4]["polygonIndices"].get< std::vector<int> >();

    // Extract points.
    pcl::PointCloud<pcl::PointNormal>::Ptr pExtracted ( new pcl::PointCloud<pcl::PointNormal> );
    pcu::extract_points<pcl::PointNormal, int>( pInput, pExtracted, indices );

    // Save points.
    pcu::write_point_cloud<pcl::PointNormal>(argv[3], pExtracted, false);

    return 0;
}
