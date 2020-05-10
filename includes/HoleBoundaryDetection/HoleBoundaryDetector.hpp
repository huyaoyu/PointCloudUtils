//
// Created by yaoyu on 3/20/20.
//

#ifndef POINTCLOUDUTILS_HOLEBOUNDARYDETECTOR_HPP
#define POINTCLOUDUTILS_HOLEBOUNDARYDETECTOR_HPP

#include <fstream>
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

class IndexSet {
public:
    typedef enum {
        UNDEFINED = 0,
        CIRCLE,
        OPEN
    }BIType_t;

public:
    IndexSet() : type(UNDEFINED) {}
    ~IndexSet() = default;

    void clear() {
        indices.clear();
        type = UNDEFINED;
    }

    void copy_indices( const std::vector<int> &extIndices ) {
        assert( extIndices.size() > 0 );

        // Resize.
        indices.resize( extIndices.size() );

        // Copy.
        std::copy( extIndices.begin(), extIndices.end(), indices.begin() );
    }

public:
    BIType_t type;
    std::vector<int> indices;
};

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
    /**
     * Set the two corner points of the current point cloud section.
     * @param corners A 6-element vector. (x0, y0, z0, x1, y1, z1).
     */
    void set_section_border_corners( const std::vector<float> &corners );
    void set_section_border_threshold( float t );
    void set_normal_view_point(float x, float y, float z);
    void set_equivalent_normal_averaging_limit(int limit);

    void process();

    void create_rgb_representation_by_criteria(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput);
    void create_rgb_representation_by_boundary_candidates( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput );
    void create_rgb_representation_by_disjoint_candidates( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOutput );

    PC_t::Ptr get_equivalent_normal();

    void write_disjoint_sets_and_normal_as_json( const std::string& fn );

protected:
    void build_proximity_graph();
    void compute_criteria();
    void coherence_filter( std::vector<bool>& vbFlag, std::vector<int>& candidates );
    void coherence_filter();
    void section_border_filter();
    void make_disjoint_boundary_candidates();
    void find_circles();
    void compute_centroid_and_equivalent_normal();

    template <typename T>
    void make_plane_coefficients( const pcl::PointNormal& pn,
            T& a, T& b, T& c, T& d );

    float criterion_value( float ac, float hc, float sc );
    bool criterion_over_threshold( float ac, float hc, float sc );
    void find_candidates_by_criteria( std::vector<bool>& vbFlag, std::vector<int>& candidates );

    bool is_near_border(const P_t &point);

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

    bool flagSectionBorder;
    Eigen::MatrixXf sectionBorder; // 4x2 matrix stores the two conner points of a bounding box.
    float sectionBorderThreshold;

    std::vector<int> boundaryCandidates;
    std::vector< std::vector<int> > disjointPointSets;
    std::vector< IndexSet > disjointBoundaryCandidates;

    int equivalentNormalAveragingLimit;
    EIGEN_ALIGN16 Eigen::Vector4f normalViewPoint;
    PC_t::Ptr pEquivalentNormal;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template < typename T >
void HBDetector::make_plane_coefficients(const pcl::PointNormal &pn, T& a, T& b, T& c, T& d) {
    // https://brilliant.org/wiki/3d-coordinate-geometry-equation-of-a-plane/
    a = pn.normal_x;
    b = pn.normal_y;
    c = pn.normal_z;
    d = -( a * pn.x + b * pn.y + c * pn.z );
}

void read_equivalent_normal_from_json( const std::string& fn,
        pcl::PointCloud<pcl::PointNormal>::Ptr normal,
        std::vector<std::vector<int>>& sets );

} // The namespace pcu

#endif //POINTCLOUDUTILS_HOLEBOUNDARYDETECTOR_HPP
