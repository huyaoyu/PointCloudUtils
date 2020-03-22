//
// Created by yaoyu on 3/21/20.
//

#ifndef POINTCLOUDUTILS_BOUNDARYCRITERION_HPP
#define POINTCLOUDUTILS_BOUNDARYCRITERION_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>

#include "HoleBoundaryDetection/ProximityGraph.hpp"
#include "Profiling/SimpleTime.hpp"

namespace pcu
{

template < typename pT >
class BoundaryCriterion {
public:
    BoundaryCriterion() = default;
    ~BoundaryCriterion() = default;

    void set_point_cloud( typename pcl::PointCloud<pT>::Ptr& ppc) {
        pInputCloud = ppc;
    }

    void set_proximity_graph( ProximityGraph<pT>* ppg ) {
        assert( ppg != nullptr );

        pProximityGraph = ppg;
    }

    template < typename realT >
    void compute( Eigen::MatrixX<realT>& criteria );

protected:
    void get_neighbor_points(int idx,
            const typename ProximityGraph<pT>::Neighbors_t& neighbors,
            typename pcl::PointCloud<pT>::Ptr& neighborPoints);

    template < typename realT >
    realT compute_angle_criterion(const pT& point, const typename pcl::PointCloud<pT>::Ptr& neighborPoints);

protected:
    typename pcl::PointCloud<pT>::Ptr pInputCloud;
    ProximityGraph<pT>* pProximityGraph;
};

template < typename pT >
void BoundaryCriterion<pT>::get_neighbor_points(int idx,
        const typename ProximityGraph<pT>::Neighbors_t& neighbors, typename pcl::PointCloud<pT>::Ptr& neighborPoints){
    // Copy the neighbor index into a pcl::PointIndices object.
    pcl::PointIndices::Ptr neighborsIdx ( new pcl::PointIndices() );
    neighborsIdx->indices.resize( neighbors.size() );
    std::copy( neighbors.begin(), neighbors.end(), neighborsIdx->indices.begin() );

    pcl::ExtractIndices<pT> extract;
    extract.setInputCloud(pInputCloud);
    extract.setIndices(neighborsIdx);
    extract.setNegative(false);
    extract.filter( *neighborPoints );
}

template < typename pT, typename realT >
static void find_a_vector( const pT& point, const typename pcl::PointCloud<pT>::Ptr& points,
        Eigen::Vector3<realT>& v, realT sDistThres=0.0001 ) {
    realT diff[3];
    auto maxSDist = static_cast<realT>(0); // Max squared distance.
    auto sDist    = static_cast<realT>(0);
    int furthestIdx = -1;

    for ( int i = 0; i < points->size(); ++i ) {
        diff[0] = (*points)[i].x - point.x;
        diff[1] = (*points)[i].y - point.y;
        diff[2] = (*points)[i].z - point.z;

        sDist = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];

        if ( sDist >= sDistThres ) {
            furthestIdx = i;
            break;
        }

        if ( sDist > maxSDist ) {
            maxSDist = sDist;
            furthestIdx = i;
        }
    }

    v(0) = points->at(furthestIdx).x - point.x;
    v(1) = points->at(furthestIdx).y - point.y;
    v(2) = points->at(furthestIdx).z - point.z;

    // Normalize.
    v.normalize();
}

template < typename pT, typename realT >
static void find_a_vector_on_plane( const pT& point,
        realT a, realT b, realT c, realT d,
        Eigen::Vector3<realT>& v){
    auto t = static_cast<realT>( 1.0 * d / ( a + b + c ) );

    v(0) = point.x + t;
    v(1) = point.y + t;
    v(2) = point.z + t;

    // Normalize.
    v.normalize();
}

template < typename pT, typename realT >
static void project_2_plane(
        const pT& point,
        const typename pcl::PointCloud<pT>::Ptr& points,
        typename pcl::PointCloud<pT>::Ptr& projected,
        Eigen::Vector3<realT>& vectorInPlane ) {
    // The coefficients.
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients() );
    coefficients->values.resize(4);
    coefficients->values[0] = point.normal_x;
    coefficients->values[1] = point.normal_y;
    coefficients->values[2] = point.normal_z;
    coefficients->values[3] = -(
            point.x * point.normal_x +
            point.y * point.normal_y +
            point.z * point.normal_z );

    // Filter.
    pcl::ProjectInliers<pT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(points);
    proj.setModelCoefficients(coefficients);
    proj.filter(*projected);

    // Find a vector in the plane.
    if ( coefficients->values[3] > 0.001 ) {
        find_a_vector_on_plane<pT, realT>(point,
                coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3],
                vectorInPlane );
    } else {
        find_a_vector<pT, realT>(point, projected, vectorInPlane);
    }
}

template < typename pT >
template < typename realT >
realT BoundaryCriterion<pT>::compute_angle_criterion(const pT& point, const typename pcl::PointCloud<pT>::Ptr& neighborPoints) {
    auto criterion = static_cast<realT>(0);
    // Project all the neighbor points to the local plane.
    typename pcl::PointCloud<pT>::Ptr projected ( new pcl::PointCloud<pT> );
    Eigen::Vector3<realT> fx;
    project_2_plane<pT>(point, neighborPoints, projected, fx);

    // Compute local frame normalized base vectors fy.
    Eigen::Vector3<realT> fz;
    fz << point.normal_x, point.normal_y, point.normal_z;
    Eigen::Vector3<realT> fy = fz.cross(fx);

    // Compute transform matrix.

    // Transform the projected neighbor points to the local frame.

    // Compute all angles.

    // Sort angle.

    // Angle criterion.

    return criterion;
}

template < typename pT >
template < typename realT >
void BoundaryCriterion<pT>::compute(Eigen::MatrixX<realT>& criteria) {
    assert(pInputCloud.get() != nullptr);
    assert(pProximityGraph != nullptr);

    QUICK_TIME_START(te)
    // Initialize the criteria.
    criteria = Eigen::MatrixXf::Zero( pInputCloud->size(), 3 );

    typename pcl::PointCloud<pT>::Ptr neighborPoints (new pcl::PointCloud<pT>);

    auto ac = static_cast<realT>(0); // Angle criterion.
    auto hc = static_cast<realT>(0); // Half-disc criterion.
    auto sc = static_cast<realT>(0); // Shape criterion.

    // Loop over all the points in the point cloud.
    for ( int i = 0; i < pInputCloud->size(); ++i ) {
        // Get the neighbors.
        typename ProximityGraph<pT>::Neighbors_t& neighbors =
                pProximityGraph->get_neighbors(i);

        // Get the sub-set of points.
        neighborPoints->clear();
        get_neighbor_points(i, neighbors, neighborPoints);

        // Compute the angle criterion.
        ac = compute_angle_criterion<realT>( (*pInputCloud)[i], neighborPoints );

        // Compute the half-disk criterion.

        // Compute the shape criterion.

        // Update the criteria.
        criteria(i, 0) = ac;
        criteria(i, 1) = hc;
        criteria(i, 2) = sc;
    }

    QUICK_TIME_END(te)

    std::cout << "Compute criteria in " << te << "ms. " << std::endl;
}

} // Namespace pcu.

#endif //POINTCLOUDUTILS_BOUNDARYCRITERION_HPP
