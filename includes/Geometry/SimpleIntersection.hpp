//
// Created by yaoyu on 4/22/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_GEOMETRY_SIMPLEINTERSECTION_HPP
#define POINTCLOUDUTILS_INCLUDES_GEOMETRY_SIMPLEINTERSECTION_HPP

#include <cmath>
#include <iostream>

#include <Eigen/Dense>

/**
 * Compute the intersection of a line and a plane.
 *
 * All input Eigen matrices must be column vectors with the same row numbers.
 *
 * @tparam Derived The underlying Eigen matrix type. All input argument has the same matrix type.
 * @param lineP Point on the line.
 * @param lineD Direction vector of the line.
 * @param planeP Point ont he plane.
 * @param planeN Normal vector of the plane.
 * @param intersection The output intersection point.
 * @return true if an intersection could be found. false if the line is parallel to the plane.
 */
template < typename Derived0, typename Derived1, typename Derived2, typename Derived3, typename Derived4 >
bool line_plane_intersection(
        const Eigen::MatrixBase<Derived1> &lineP,
        const Eigen::MatrixBase<Derived2> &lineD,
        const Eigen::MatrixBase<Derived3> &planeP,
        const Eigen::MatrixBase<Derived4> &planeN,
        Eigen::MatrixBase<Derived0> &intersection ) {
    assert( 1 == lineP.cols() );
    assert( 1 == lineD.cols() );
    assert( 1 == planeP.cols() );
    assert( 1 == planeN.cols() );

    // Dot product between the line direction and the normal of the plane.
    const typename Derived0::Scalar d = lineD.transpose() * planeN;

    if ( std::abs( d ) < 1e-6 ) {
        return false;
    }

    const typename Derived0::Scalar n = planeN.transpose() * ( planeP - lineP );
    const typename Derived0::Scalar f = n/d;

    intersection = lineP + f * lineD;

    return true;
}

/**
 * Compute the intersection of a line and a plane considering the direction of the line.
 *
 * All input Eigen matrices must be column vectors with the same row numbers.
 *
 * @tparam Derived The underlying Eigen matrix type. All input argument has the same matrix type.
 * @param lineP Point on the line.
 * @param lineD Direction vector of the line.
 * @param planeP Point ont he plane.
 * @param planeN Normal vector of the plane.
 * @param intersection The output intersection point.
 * @return true if an intersection could be found. false if the line is parallel to the plane.
 */
template < typename Derived0, typename Derived1, typename Derived2, typename Derived3, typename Derived4 >
bool line_plane_directed_intersection(
        const Eigen::MatrixBase<Derived1> &lineP,
        const Eigen::MatrixBase<Derived2> &lineD,
        const Eigen::MatrixBase<Derived3> &planeP,
        const Eigen::MatrixBase<Derived4> &planeN,
        Eigen::MatrixBase<Derived0> &intersection ) {
    assert( 1 == lineP.cols() );
    assert( 1 == lineD.cols() );
    assert( 1 == planeP.cols() );
    assert( 1 == planeN.cols() );

    // Dot product between the line direction and the normal of the plane.
    const typename Derived0::Scalar d = lineD.transpose() * planeN;

    if ( std::abs( d ) < 1e-6 ) {
        return false;
    }

    const typename Derived0::Scalar n = planeN.transpose() * ( planeP - lineP );
    const typename Derived0::Scalar f = n/d;

    if ( f < 0 ) {
        return false;
    }

    intersection = lineP + f * lineD;

    return true;
}

#endif //POINTCLOUDUTILS_INCLUDES_GEOMETRY_SIMPLEINTERSECTION_HPP
