//
// Created by yaoyu on 3/21/20.
//

#ifndef POINTCLOUDUTILS_BOUNDARYCRITERION_HPP
#define POINTCLOUDUTILS_BOUNDARYCRITERION_HPP

#include <algorithm>    // std::sort
#include <cmath>
#include <iostream>
#include <numeric>      // std::iota
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
#include "PCCommon/extraction.hpp"
#include "Profiling/SimpleTime.hpp"
#include "Visualization/ListPoints.hpp"

namespace pcu
{

template < typename pT, typename realT >
class BoundaryCriterion {
public:
    typedef typename ProximityGraph<pT>::Index_t Index_t;
public:
    BoundaryCriterion(): neighborDistanceLimit(1e-4), halfDiscSigmaFactor(1.0), shapeSigmaFactor(1.0) {
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

    void set_neighbor_distance_limit(realT d) {
        assert(d > 0);
        neighborDistanceLimit = d;
    }

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

    void compute( Eigen::MatrixX<realT>& criteria,
            Eigen::MatrixX<Index_t>& maxAngleNeighbors,
            Eigen::MatrixX<realT>& rp, Index_t startIdx=0 );

protected:
    void get_neighbor_points(
            const std::vector<Index_t>& neighbors,
            typename pcl::PointCloud<pT>::Ptr& neighborPoints);

    realT shape_criterion( const typename pcl::PointCloud<pT>::Ptr& points );

    void compute_criteria(const pT& point,
                          const typename pcl::PointCloud<pT>::Ptr& neighborPoints,
                          typename std::vector<Index_t>& vNeighbors,
                          realT& ac, realT& hc, realT& sc,
                          std::vector<Index_t>& maxAngleNeighbors,
                          realT& avgDistance );

protected:
    typename pcl::PointCloud<pT>::Ptr pInputCloud;
    ProximityGraph<pT>* pProximityGraph;

    realT neighborDistanceLimit;

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

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template < typename pT, typename realT >
void BoundaryCriterion<pT, realT>::get_neighbor_points(
        const std::vector<Index_t>& neighbors, typename pcl::PointCloud<pT>::Ptr& neighborPoints){
    extract_points<pT, Index_t>( pInputCloud, neighborPoints, neighbors );
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
        realT& avgDistance,
        realT distLimit,
        std::vector<int>& validIdx) {
    const auto n = points->size();

    angles.resize( n );
    distances.resize( n );
    validIdx.resize( n );

    auto x       = static_cast<realT>(0);
    auto y       = static_cast<realT>(0);
    auto accDist = static_cast<realT>(0);
    auto dist    = static_cast<realT>(0);
    int validCount = 0;

    for ( int i = 0; i < n; ++i ) {
        x = points->at(i).x;
        y = points->at(i).y;

        dist = std::sqrt( x*x + y*y );

        if ( dist < distLimit ) {
            continue;
        }

        accDist += dist;

        distances[validCount] = dist;
        angles[validCount]    = static_cast<realT>( std::atan2( y, x ) );
        validIdx[validCount]  = i;

        validCount++;
    }

    avgDistance = static_cast<realT>( 1.0 * accDist / validCount );

    // Resize.
    angles.resize(validCount);
    distances.resize(validCount);
    validIdx.resize(validCount);
}

template < typename realT >
static realT checked_angle(realT a, realT pi) {
    return ( a > pi ) ? ( 2*pi - a ) : a ;
}

template < typename realT, typename iT >
static void make_angle_pairs( const std::vector<realT>& angles,
        const std::vector<iT>& vNeighbors,
        std::vector< std::pair<realT, iT> >& anglePairs ) {
    const int n = angles.size();

    assert( n != 0 );
    assert( n == vNeighbors.size() );
    anglePairs.resize( n );

    for ( int i = 0; i < n; ++i ) {
        anglePairs[i] = std::make_pair( angles[i], vNeighbors[i] );
    }
}

/**
 * It is assumed that the values in soredAngles are sorted in ascending order.
 * @tparam realT
 * @param sortedAnglePairs std::pair's, first is the angle, second is the original index.
 * @param maxAngleIndices A vector stores the indices of the two points that make the maximum angle. The indices are pair.second values.
 * @return Then angle-criterion.
 */
template < typename realT, typename iT >
static realT angle_criterion( const std::vector<std::pair<realT, iT>>& sortedAnglePairs,
        std::vector<iT>& maxAngleIndices ) {
    const auto n = sortedAnglePairs.size();
    assert( n > 1 );
    assert( maxAngleIndices.size() >= 2 );

    auto maxAngle = static_cast<realT>(0);
    auto a = static_cast<realT>(0);
    int neighbor0, neighbor1;

    const auto pi = boost::math::constants::pi<realT>();

    for ( int i = 1; i < n; ++i ) {
        a = sortedAnglePairs[i].first - sortedAnglePairs[i - 1].first;

//        a = checked_angle(a, pi);

        if ( a > maxAngle ) {
            maxAngle = a;
            neighbor0 = i-1;
            neighbor1 = i;
        }
    }

//    a = checked_angle(sortedAnglePairs[n-1] - sortedAnglePairs[0], pi);
    a = 2*pi - (sortedAnglePairs[n - 1].first - sortedAnglePairs[0].first );

    if ( a > maxAngle ) {
        maxAngle = a;
        neighbor0 = 0;
        neighbor1 = n-1;
    }

    maxAngleIndices[0] = sortedAnglePairs[neighbor0].second;
    maxAngleIndices[1] = sortedAnglePairs[neighbor1].second;

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

template < typename T, typename iT >
static void copy_values( const T& from, T& to, const iT& indices ) {
    if ( &from == &to ) {
        std::stringstream ss;
        ss << "The address of 'from' and 'to' are the same. ";
        throw( std::runtime_error( ss.str() ) );
    }

    const int n = indices.size();

    to.resize( n );

    for ( int i = 0; i < n; ++i ) {
        to[i] = from[ indices[i] ];
    }
}

template < typename pT, typename realT >
void BoundaryCriterion<pT, realT>::compute_criteria(
        const pT& point,
        const typename pcl::PointCloud<pT>::Ptr& neighborPoints,
        typename std::vector<Index_t>& vNeighbors,
        realT& ac, realT& hc, realT& sc,
        std::vector<Index_t>& maxAngleNeighbors,
        realT& avgDistance ) {
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
//    for ( const auto& vn : vNeighbors ) std::cout << vn << ", "; std::cout << std::endl;
//    pcu::list_points<pT>( neighborPoints, "neighborPoints: " );
//    pcu::list_points<pT>( projected, "projected: " );
//    pcu::list_points<pT>( transformed, "transformed: " );
//    throw(std::runtime_error("Test use!"));

    // Compute all angles.
    std::vector<realT> angles;
    std::vector<realT> distances;
    std::vector<int> validIdx;

    compute_angles_and_distances_2D<pT, realT>(
            transformed, angles, distances, avgDistance,
            neighborDistanceLimit, validIdx);

    // Copy the valid values.
    std::vector<Index_t> vNeighborsFiltered;
    copy_values(vNeighbors, vNeighborsFiltered, validIdx);

    typename pcl::PointCloud<pT>::Ptr transformedFiltered ( new pcl::PointCloud<pT> );
    typename pcl::PointCloud<pT>::Ptr transformedNeighborsFiltered ( new pcl::PointCloud<pT> );
    extract_points<pT, Index_t>( transformed, transformedFiltered, validIdx );
    extract_points<pT, Index_t>( transformedNeighbors, transformedNeighborsFiltered, validIdx );

//    // Test use.
//    std::cout << "Angles before sorting." << std::endl;
//    for ( const auto& a : angles ) {
//        std::cout << a << ", ";
//    }
//    std::cout << std::endl;

    // Make angle-pairs.
    std::vector< std::pair<realT, Index_t> > anglePairs;
    make_angle_pairs( angles, vNeighborsFiltered, anglePairs );

    // Sort angle.
    std::sort( anglePairs.begin(), anglePairs.end() );

//    // Test use.
//    std::cout << "Angles after sorting." << std::endl;
//    for ( const auto& a : angles ) {
//        std::cout << a << ", ";
//    }
//    std::cout << std::endl;

    // Angle criterion.
    ac = angle_criterion(anglePairs, maxAngleNeighbors);

//    // Test use.
//    std::cout << "criterion = " << criterion << std::endl;
//    throw(std::runtime_error("Test!"));

    // Half-disc criterion.
    hc = half_disc_criterion<pT, realT>( transformedFiltered, distances, avgDistance, halfDiscSigmaFactor );

//    // Test use.
//    throw(std::runtime_error("Test!"));

    // Shape criterion. This is a protected member-function.
    sc = shape_criterion( transformedNeighborsFiltered );

//    // Test use.
//    std::cout << "point = " << point << std::endl;
//    pcu::list_points<pT>( transformedNeighbors, "transformedNeighbors: " );

    // Copy the filtered neighbors back to vNeighbors.
    vNeighbors.resize( vNeighborsFiltered.size() );
    std::copy( vNeighborsFiltered.begin(), vNeighborsFiltered.end(), vNeighbors.begin() );
}

template < typename pT, typename realT >
void BoundaryCriterion<pT, realT>::compute(
        Eigen::MatrixX<realT>& criteria,
        Eigen::MatrixX<Index_t>& maxAngleNeighbors,
        Eigen::MatrixX<realT>& rp,
        Index_t startIdx ) {
    assert(pInputCloud.get() != nullptr);
    assert(pProximityGraph != nullptr);
    assert( startIdx >=0 && startIdx < pInputCloud->size() );

    QUICK_TIME_START(te)
    // Initialize the criteria.
    criteria = Eigen::MatrixXf::Zero( pInputCloud->size(), 3 );
    maxAngleNeighbors = Eigen::MatrixXi::Zero( pInputCloud->size(), 2 );
    rp = Eigen::MatrixXf::Zero( pInputCloud->size(), 1 );

    typename pcl::PointCloud<pT>::Ptr neighborPoints (new pcl::PointCloud<pT>);

    auto ac = static_cast<realT>(0); // Angle criterion.
    auto hc = static_cast<realT>(0); // Half-disc criterion.
    auto sc = static_cast<realT>(0); // Shape criterion.
    auto ad = static_cast<realT>(0); // Average distance.

    std::vector<Index_t> maxAngleNeighborIndex(2);

    // Loop over all the points in the point cloud.
    for ( Index_t i = startIdx; i < pInputCloud->size(); ++i ) {
        // Get the neighbors.
        typename ProximityGraph<pT>::Neighbors_t& neighbors =
                pProximityGraph->get_neighbors(i);

        // Convert the neighbors to std::vector type;
        std::vector<Index_t> vNeighbors( neighbors.begin(), neighbors.end() );

        // Get the sub-set of points.
        neighborPoints->clear();
        get_neighbor_points( vNeighbors, neighborPoints);

        // Compute all the criteria.
        compute_criteria( (*pInputCloud)[i], neighborPoints, vNeighbors,
                ac, hc, sc, maxAngleNeighborIndex, ad );

        maxAngleNeighbors(i, 0) = maxAngleNeighborIndex[0];
        maxAngleNeighbors(i, 1) = maxAngleNeighborIndex[1];

        // Update the criteria.
        criteria(i, 0) = ac;
        criteria(i, 1) = hc;
        criteria(i, 2) = sc;

        rp(i, 0) = ad;

        // Update the proximity graph.
        neighbors.clear();
        std::copy( vNeighbors.begin(), vNeighbors.end(), std::inserter(neighbors, neighbors.end()) );
    }

    QUICK_TIME_END(te)

    std::cout << "Compute criteria in " << te << "ms. " << std::endl;
}

} // Namespace pcu.

#endif //POINTCLOUDUTILS_BOUNDARYCRITERION_HPP
