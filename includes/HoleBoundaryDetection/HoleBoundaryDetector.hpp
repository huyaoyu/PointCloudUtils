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

#include "Graph/Edge.hpp"
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
    void set_criterion_computation_start_index(int idx);
    void set_criterion_params(float fA, float fH, float fS, float t);

    void process();

    void create_rgb_representation_by_criteria(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput);
    void create_rgb_representation_by_boundary_candidates( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput );
    void create_rgb_representation_by_disjoint_candidates( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput );

protected:
    void build_proximity_graph();
    void compute_criteria();
    void coherence_filter( std::vector<bool>& vbFlag, std::vector<int>& candidates );
    void coherence_filter();
    void make_disjoint_boundary_candidates();

    template <typename T>
    void make_plane_coefficients( const pcl::PointNormal& pn,
            T& a, T& b, T& c, T& d );

    float criterion_value( float ac, float hc, float sc );
    bool criterion_over_threshold( float ac, float hc, float sc );
    void find_candidates_by_criteria( std::vector<bool>& vbFlag, std::vector<int>& candidates );

    void map_indices_in_proximity_graph( ProximityGraph<P_t>& pg, const std::vector<int>& reference );
    void create_edges_from_points( const std::vector<int>& referenceIndices,
            const ProximityGraph<P_t>& pg,
            std::vector<Edge<int, float>>& edges );
    void make_disjoint_sets_from_edges(
            const std::vector<Edge<int, float>>& edges,
            const std::vector<int>& references,
            std::vector<std::vector<int>>& disjointSets );

protected:
    PC_t::Ptr pInput;

    KRProximityGraph<P_t> proximityGraph;
    int    pgK;
    double pgR;
    int    pgSDB; // Show detail base.

    int criteriaComputationStartIdx;
    Eigen::MatrixXf criteria;
    Eigen::MatrixXi maxAngleNeighbors;
    Eigen::MatrixXf rp; // The average distance.

    float factorAngleCriterion;
    float factorHalfDiscCriterion;
    float factorShapeCriterion;
    float criterionThreshold;

    std::vector<int> boundaryCandidates;

    std::vector< std::vector<int> > disjointBoundaryCandidates;
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
