//
// Created by yaoyu on 3/21/20.
//

#ifndef POINTCLOUDUTILS_BOUNDARYCRITERION_HPP
#define POINTCLOUDUTILS_BOUNDARYCRITERION_HPP

#include <algorithm>    // std::sort
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h> // Covariance matrix.
#include <pcl/common/eigen.h> // Eigen values.
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>

#include "Geometry/TransformHelpers.hpp"
#include "HoleBoundaryDetection/ProximityGraph.hpp"
#include "Profiling/SimpleTime.hpp"
#include "Visualization/ListPoints.hpp"

namespace pcu
{

template < typename pT, typename realT >
class BoundaryCriterion {
public:
    BoundaryCriterion(): halfDiscSigmaFactor(1.0), shapeSigmaFactor(1.0) {
        const auto oneSecond = static_cast<realT>(1.0/2);
        const auto oneThird  = static_cast<realT>(1.0/3);
        const auto oneFourth = static_cast<realT>(1.0/4);

        barycentricPointInterior << 1, 0, 0;
        barycentricPointLine     << 0, 1, 0;
        barycentricPointCorner   << 0, 0, 1;
        barycentricPointCenter   << oneThird, oneThird, oneThird;

        barycentricTransformMat << oneSecond, oneSecond, oneThird,
                                   oneSecond, oneFourth, oneThird,
                                           0, oneFourth, oneThird;

        barycentricTransformMat = barycentricTransformMat.inverse().eval();

        EIGEN_ALIGN16 Eigen::Vector3<realT> valueBoundary;
        valueBoundary << 2*oneThird, oneThird, 0;

        barycentricPointBoundary = barycentricTransformMat * valueBoundary;

        shapeSigma = shapeSigmaFactor * ( barycentricPointInterior - barycentricPointBoundary ).norm();
        shapeSigma2 = shapeSigma * shapeSigma;
    }

    ~BoundaryCriterion() = default;

    void set_half_disc_sigma_factor(realT f) {
        assert( f > 0 );
        halfDiscSigmaFactor = f;
    }

    void set_shape_sigma_factor(realT f) {
        assert( f > 0 );
        shapeSigmaFactor = f;
        shapeSigma  = shapeSigmaFactor * ( barycentricPointLine - barycentricPointBoundary ).norm();
        shapeSigma2 = shapeSigma * shapeSigma;
    }

    void set_point_cloud( typename pcl::PointCloud<pT>::Ptr& ppc) {
        pInputCloud = ppc;
    }

    void set_proximity_graph( ProximityGraph<pT>* ppg ) {
        assert( ppg != nullptr );

        pProximityGraph = ppg;
    }

    void compute( Eigen::MatrixX<realT>& criteria, int startIdx=0 );

protected:
    void get_neighbor_points(int idx,
            const typename ProximityGraph<pT>::Neighbors_t& neighbors,
            typename pcl::PointCloud<pT>::Ptr& neighborPoints);

    realT shape_criterion( const typename pcl::PointCloud<pT>::Ptr& points );

    void compute_criteria(const pT& point,
                          const typename pcl::PointCloud<pT>::Ptr& neighborPoints,
                          realT& ac, realT& hc, realT& sc );

protected:
    typename pcl::PointCloud<pT>::Ptr pInputCloud;
    ProximityGraph<pT>* pProximityGraph;

    realT halfDiscSigmaFactor;

    EIGEN_ALIGN16 Eigen::Vector3<realT> barycentricPointLine;
    EIGEN_ALIGN16 Eigen::Vector3<realT> barycentricPointInterior;
    EIGEN_ALIGN16 Eigen::Vector3<realT> barycentricPointCorner;
    EIGEN_ALIGN16 Eigen::Vector3<realT> barycentricPointBoundary;
    EIGEN_ALIGN16 Eigen::Vector3<realT> barycentricPointCenter;

    EIGEN_ALIGN16 Eigen::Matrix3<realT> barycentricTransformMat; // The matrix apply to a value vector to get the Barycentric coordinate vector.

    realT shapeSigmaFactor;
    realT shapeSigma;
    realT shapeSigma2;
};

template < typename pT, typename realT >
void BoundaryCriterion<pT, realT>::get_neighbor_points(int idx,
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

template < typename pT, typename realT >
static void compute_angles_and_distances_2D(
        const typename pcl::PointCloud<pT>::Ptr& points,
        std::vector<realT>& angles,
        std::vector<realT>& distances,
        realT& avgDistance ) {
    angles.resize( points->size() );
    distances.resize( points->size() );

    auto x = static_cast<realT>(0);
    auto y = static_cast<realT>(0);
    auto accDist = static_cast<realT>(0);
    auto dist = static_cast<realT>(0);

    for ( int i = 0; i < points->size(); ++i ) {
        x = points->at(i).x;
        y = points->at(i).y;

        angles[i] = static_cast<realT>( std::atan2( y, x ) );
        dist = std::sqrt( x*x + y*y );
        accDist += dist;
        distances[i] = dist;
    }

    avgDistance = static_cast<realT>( 1.0 * accDist / points->size() );
}

template < typename realT >
static realT checked_angle(realT a, realT pi) {
    return ( a > pi ) ? ( 2*pi - a ) : a ;
}

/**
 * It is assumed that the values in soredAngles are sorted in ascending order.
 * @tparam realT
 * @param sortedAngles
 * @return
 */
template < typename realT >
static realT angle_criterion( const std::vector<realT>& sortedAngles ) {
    const auto n = sortedAngles.size();
    assert( n > 1 );

    auto maxAngle = static_cast<realT>(0);
    auto a = static_cast<realT>(0);

    const auto pi = boost::math::constants::pi<realT>();

    for ( int i = 1; i < n; ++i ) {
        a = sortedAngles[i] - sortedAngles[i-1];

//        a = checked_angle(a, pi);

        if ( a > maxAngle ) {
            maxAngle = a;
        }
    }

//    a = checked_angle(sortedAngles[n-1] - sortedAngles[0], pi);
    a = 2*pi - ( sortedAngles[n-1] - sortedAngles[0] );

    if ( a > maxAngle ) {
        maxAngle = a;
    }

//    // Test use.
//    std::cout << "maxAngle = " << maxAngle << std::endl;

    const auto f = static_cast<realT>( 2.0*pi / n );

    return std::min( ( maxAngle - f ) / ( pi - f ), static_cast<realT>(1) );
}

template < typename pT, typename realT >
static realT half_disc_criterion(
        const typename pcl::PointCloud<pT>::Ptr& points,
        const std::vector<realT>& distances,
        realT avgDistance,
        realT sigmaFactor=1.0) {
    const auto n = points->size();

    assert( n > 0 );
    assert( distances.size() == n );

    const auto sigma  = sigmaFactor * avgDistance;
    const auto sigma2 = sigma * sigma;

    auto g    = static_cast<realT>(0);
    auto accX = static_cast<realT>(0);
    auto accY = static_cast<realT>(0);
    auto accZ = static_cast<realT>(0);
    auto accW = static_cast<realT>(0);
    auto d    = static_cast<realT>(0);

    for ( int i = 0; i < n; ++i ) {
        d = distances[i];

        g = static_cast<realT>( std::exp( -d*d / sigma2 ) );

        accX += g * points->at(i).x;
        accY += g * points->at(i).y;
        accZ += g * points->at(i).z;
        accW += g;
    }

    accX /= accW;
    accY /= accW;
    accZ /= accW;

    const auto pi = boost::math::constants::pi<realT>();

    return std::min(
            static_cast<realT>( std::sqrt( accX * accX + accY * accY + accZ * accZ ) / ( 4.0/3/pi * avgDistance ) ),
            static_cast<realT>(1) );
}

template < typename pT, typename realT >
static void compute_lambda( const typename pcl::PointCloud<pT>::Ptr& points,
        Eigen::Vector3<realT>& vLambda ) {
    // Compute the covariance matrix.
    EIGEN_ALIGN16 Eigen::Matrix3<realT> covMat = Eigen::Matrix3<realT>::Zero();

    if ( 0 == pcl::computeCovarianceMatrix( *points, covMat ) ) {
        std::stringstream ss;
        ss << "Compute covariance matrix failed.";
        throw( std::runtime_error(ss.str()) );
    }

    // Compute the eigen values.
    EIGEN_ALIGN16 Eigen::Vector3<realT> eigenValues;
    pcl::eigen33( covMat, eigenValues ); // Results will be in increasing order.

    vLambda << eigenValues(2), eigenValues(1), eigenValues(0);
    vLambda /= eigenValues.sum();
}

template < typename pT, typename realT >
realT BoundaryCriterion<pT, realT>::shape_criterion(const typename pcl::PointCloud<pT>::Ptr& points) {
    // Compute the lambda vector.
    EIGEN_ALIGN16 Eigen::Vector3<realT> vLambda = Eigen::Vector3<realT>::Zero();
    compute_lambda<pT, realT>( points, vLambda );

    // Compute the Barycentric coordinate.
    EIGEN_ALIGN16 Eigen::Vector3<realT> coor = barycentricTransformMat * vLambda;

//    // Test use.
//    std::cout << "barycentricPointInterior = " << std::endl << barycentricPointInterior << std::endl;
//    std::cout << "barycentricPointLine = "     << std::endl << barycentricPointLine << std::endl;
//    std::cout << "barycentricPointCorner = "   << std::endl << barycentricPointCorner << std::endl;
//    std::cout << "barycentricPointBoundary = " << std::endl << barycentricPointBoundary << std::endl;
//    std::cout << "barycentricTransformMat = "  << std::endl << barycentricTransformMat << std::endl;
//    std::cout << "shapeSigma = " << shapeSigma << std::endl;
//    std::cout << "vLambda = " << std::endl << vLambda << std::endl;
//    std::cout << "coor = "    << std::endl << coor << std::endl;

    // Distance between the current coordinate and the point of "boundary type".
    const realT dist = ( barycentricPointBoundary - coor ).norm();

    return std::exp( -dist*dist / shapeSigma2 );
}

template < typename pT, typename realT >
void BoundaryCriterion<pT, realT>::compute_criteria(
        const pT& point, const typename pcl::PointCloud<pT>::Ptr& neighborPoints,
        realT& ac, realT& hc, realT& sc ) {
    auto criterion = static_cast<realT>(0);

//    // Test use: add the current point to the set of neighbor points.
//    neighborPoints->push_back(point);

    // Project all the neighbor points to the local plane.
    typename pcl::PointCloud<pT>::Ptr projected ( new pcl::PointCloud<pT> );
    Eigen::Vector3<realT> fx;
    project_2_plane<pT>(point, neighborPoints, projected, fx);

    // Compute local frame normalized base vectors fy.
    Eigen::Vector3<realT> fz;
    fz << point.normal_x, point.normal_y, point.normal_z;
    Eigen::Vector3<realT> fy = fz.cross(fx);

    // Compute transform matrix.
    Eigen::Vector3<realT> t;
    t << point.x, point.y, point.z;
    Eigen::Matrix4<realT> tm;
    make_trans_mat_by_change_of_normalized_basis(
            fx, fy, fz, t, tm );

    // Transform the projected neighbor points to the local frame.
    typename pcl::PointCloud<pT>::Ptr transformed ( new pcl::PointCloud<pT> );
    typename pcl::PointCloud<pT>::Ptr transformedNeighbors ( new pcl::PointCloud<pT> );
    pcl::transformPointCloud( *projected, *transformed, tm );
    pcl::transformPointCloud( *neighborPoints, *transformedNeighbors, tm );

//    // Test use.
//    pcu::list_points<pT>( neighborPoints, "neighborPoints: " );
//    pcu::list_points<pT>( projected, "projected: " );
//    pcu::list_points<pT>( transformed, "transformed: " );
//    throw(std::runtime_error("Test use!"));

    // Compute all angles.
    std::vector<realT> angles;
    std::vector<realT> distances;
    realT avgDistance;
    compute_angles_and_distances_2D<pT, realT>(transformed, angles, distances, avgDistance);

//    // Test use.
//    std::cout << "Angles before sorting." << std::endl;
//    for ( const auto& a : angles ) {
//        std::cout << a << ", ";
//    }
//    std::cout << std::endl;

    // Sort angle.
    std::sort( angles.begin(), angles.end() );

//    // Test use.
//    std::cout << "Angles after sorting." << std::endl;
//    for ( const auto& a : angles ) {
//        std::cout << a << ", ";
//    }
//    std::cout << std::endl;

    // Angle criterion.
    ac = angle_criterion(angles);

//    // Test use.
//    std::cout << "criterion = " << criterion << std::endl;
//    throw(std::runtime_error("Test!"));

    // Half-disc criterion.
    hc = half_disc_criterion<pT, realT>( transformed, distances, avgDistance, halfDiscSigmaFactor );

//    // Test use.
//    throw(std::runtime_error("Test!"));

    // Shape criterion. This is a protected member-function.
    sc = shape_criterion( transformedNeighbors );

//    // Test use.
//    std::cout << "point = " << point << std::endl;
//    pcu::list_points<pT>( transformedNeighbors, "transformedNeighbors: " );
}

template < typename pT, typename realT >
void BoundaryCriterion<pT, realT>::compute( Eigen::MatrixX<realT>& criteria, int startIdx ) {
    assert(pInputCloud.get() != nullptr);
    assert(pProximityGraph != nullptr);
    assert( startIdx >=0 && startIdx < pInputCloud->size() );

    QUICK_TIME_START(te)
    // Initialize the criteria.
    criteria = Eigen::MatrixXf::Zero( pInputCloud->size(), 3 );

    typename pcl::PointCloud<pT>::Ptr neighborPoints (new pcl::PointCloud<pT>);

    auto ac = static_cast<realT>(0); // Angle criterion.
    auto hc = static_cast<realT>(0); // Half-disc criterion.
    auto sc = static_cast<realT>(0); // Shape criterion.

    // Loop over all the points in the point cloud.
    for ( int i = startIdx; i < pInputCloud->size(); ++i ) {
        // Get the neighbors.
        typename ProximityGraph<pT>::Neighbors_t& neighbors =
                pProximityGraph->get_neighbors(i);

        // Get the sub-set of points.
        neighborPoints->clear();
        get_neighbor_points(i, neighbors, neighborPoints);

        // Compute all the criteria.
        compute_criteria((*pInputCloud)[i], neighborPoints,
                                ac, hc, sc);

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
