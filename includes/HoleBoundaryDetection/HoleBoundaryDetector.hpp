//
// Created by yaoyu on 3/20/20.
//

#ifndef POINTCLOUDUTILS_HOLEBOUNDARYDETECTOR_HPP
#define POINTCLOUDUTILS_HOLEBOUNDARYDETECTOR_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "Profiling/SimpleTime.hpp"

namespace pcu
{

class HBDetector {
public:
    typedef pcl::PointNormal P_t;
    typedef pcl::PointCloud<P_t> PC_t;

public:
    HBDetector();
    ~HBDetector();

    void set_point_cloud(PC_t::Ptr& pInput);

protected:
    PC_t::Ptr mpInput;
};

} // The namespace pcu

#endif //POINTCLOUDUTILS_HOLEBOUNDARYDETECTOR_HPP
