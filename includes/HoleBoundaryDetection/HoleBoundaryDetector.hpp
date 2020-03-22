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

#include "HoleBoundaryDetection/ProximityGraph.hpp"
#include "HoleBoundaryDetection/BoundaryCriterion.hpp"
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

    void set_point_cloud(PC_t::Ptr& p);

    void set_proximity_graph_params(int k, double radius, int showDetailBase=100000);

    ProximityGraph<P_t>& get_proximity_graph();

    void process();

protected:
    void build_proximity_graph();
    void compute_criteria();

    template <typename T>
    void make_plane_coefficients( const pcl::PointNormal& pn,
            T& a, T& b, T& c, T& d );

protected:
    PC_t::Ptr pInput;

    ProximityGraph<P_t> proximityGraph;
    int    pgK;
    double pgR;
    int    pgSDB; // Show detail base.

    Eigen::MatrixXf criteria;
};

template < typename T >
void HBDetector::make_plane_coefficients(const pcl::PointNormal &pn, T& a, T& b, T& c, T& d) {
    // https://brilliant.org/wiki/3d-coordinate-geometry-equation-of-a-plane/
    a = pn.normal_x;
    b = pn.normal_y;
    c = pn.normal_z;
    d = -( a * pn.x + b * pn.y + c * pn.z );
}

} // The namespace pcu

#endif //POINTCLOUDUTILS_HOLEBOUNDARYDETECTOR_HPP
