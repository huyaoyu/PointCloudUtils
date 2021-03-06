//
// Created by yaoyu on 3/25/20.
//

#ifndef POINTCLOUDUTILS_COMMON_HPP
#define POINTCLOUDUTILS_COMMON_HPP

#include <algorithm>    // std::sort
#include <cmath>
#include <iostream>
#include <numeric>      // std::iota
#include <sstream>
#include <string>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

namespace pcu
{

template < typename pT, typename rT >
void convert_pcl_2_eigen_matrix(
        const typename pcl::PointCloud<pT>::Ptr pInput,
        Eigen::MatrixX<rT> &points ) {
    const std::size_t N = pInput->size();
    assert(N > 0);

    points.resize(3, N);
    for ( std::size_t i = 0; i < N; ++i) {
        const auto& point = pInput->at(i);
        points(0, i) = point.x;
        points(1, i) = point.y;
        points(2, i) = point.z;
    }
}

template < typename pT, typename rT >
typename pcl::PointCloud<pT>::Ptr convert_eigen_matrix_2_pcl_xyz(
        const Eigen::MatrixX<rT> &mat ) {
    const std::size_t N = mat.cols();

    assert( N > 0 );

    typename pcl::PointCloud<pT>::Ptr pOutput ( new pcl::PointCloud<pT> );
    pOutput->resize( N );

    for ( std::size_t i = 0; i < N; ++i ) {
        pT p;
        p.x = mat(0, i);
        p.y = mat(1, i);
        p.z = mat(2, i);
        pOutput->at(i) = p;
    }

    return pOutput;
}

template < typename rT >
pcl::PointCloud<pcl::PointXYZ>::Ptr convert_eigen_depth_img_2_pcl_xyz(
        const Eigen::MatrixX<rT> &img ) {
    const std::size_t height = img.rows();
    const std::size_t width  = img.cols();

    assert( height > 0 );
    assert( width  > 0 );

    pcl::PointCloud<pcl::PointXYZ>::Ptr pOutput ( new pcl::PointCloud<pcl::PointXYZ> );
    pOutput->clear();

    std::size_t count = 0;

    for ( std::size_t i = 0; i < height; ++i ) {
        for ( std::size_t j = 0; j < width; ++j ) {
            if ( img(i, j) <= 0 ) {
                continue;
            }

            pcl::PointXYZ p;
            p.x = j;
            p.y = i;
            p.z = img(i, j);
            pOutput->push_back(p);
            count++;
        }
    }

    return pOutput;
}

template < typename pT, typename rT >
Eigen::Vector3<rT> create_eigen_vector3_from_xyz(const pT &point) {
    EIGEN_ALIGN16 Eigen::Vector3<rT> v;
    v << point.x, point.y, point.z;
    return v;
}

template < typename pT, typename rT >
Eigen::Vector4<rT> create_eigen_vector4_from_xyz(const pT &point) {
    EIGEN_ALIGN16 Eigen::Vector4<rT> v;
    v << point.x, point.y, point.z, static_cast<rT>(1.0);
    return v;
}

template < typename pT, typename rT >
Eigen::Vector3<rT> create_eigen_vector3_from_normal( const pT &point ) {
    EIGEN_ALIGN16 Eigen::Vector3<rT> v;
    v << point.normal_x, point.nomrl_y, point.normal_z;
    return v;
}

template < typename pT, typename rT >
Eigen::Vector4<rT> create_eigen_vector4_from_normal( const pT &point ) {
    EIGEN_ALIGN16 Eigen::Vector4<rT> v;
    v << point.normal_x, point.nomrl_y, point.normal_z, static_cast<rT>(1.0);
    return v;
}

template < typename rT >
Eigen::Matrix4<rT> create_eigen_transform_matrix_0(
        const pcl::PointXYZ &translation,
        const Eigen::Matrix3<rT> &rotation ) {
    Eigen::Matrix4<rT> transMat = Eigen::Matrix4<rT>::Identity();

    transMat.block( 0,0,3,3 ) = rotation;
//    transMat( Eigen::seq(0,2), Eigen::seq(0,2) ) = rotation;
    transMat( 0, 3 ) = translation.x;
    transMat( 1, 3 ) = translation.y;
    transMat( 2, 3 ) = translation.z;

    return transMat;
}

template < typename rT >
Eigen::Matrix4<rT> create_eigen_transform_matrix_1(
        const pcl::PointXYZ &translation,
        const Eigen::Matrix3<rT> &rotation ) {
    Eigen::Matrix4<rT> transMat = Eigen::Matrix4<rT>::Identity();

    Eigen::Vector3<rT> transVec;
    transVec << translation.x, translation.y, translation.z;

    transMat.block(0,0,3,3) = rotation.transpose();
    transMat( Eigen::seq(0,2), 3 ) = -rotation.transpose() * transVec;

    return transMat;
}

template < typename pT0, typename pT1 >
float distance_two_points( const pT0& p0, const pT1& p1 ) {
    const float d0 = p0.x - p1.x;
    const float d1 = p0.y - p1.y;
    const float d2 = p0.z - p1.z;

    return std::sqrt( d0 * d0 + d1 * d1 + d2 * d2 );
}

template < typename iT >
pcl::PointIndices::Ptr convert_vector_2_pcl_indices( const std::vector<iT> &v) {
    const auto N = v.size();

    if ( N == 0 ) {
        std::stringstream ss;
        ss << "Vector is empty. Cannot convert to pcl::PointIndices. ";
        throw( std::runtime_error( ss.str() ) );
    }

    pcl::PointIndices::Ptr pclIndices (new pcl::PointIndices() );

    pclIndices->indices.resize( N );
    std::copy(v.begin(), v.end(), pclIndices->indices.begin() );

    return pclIndices;
}

template < typename iT >
void convert_vector_2_pcl_indices( const std::vector<iT>& v, pcl::PointIndices::Ptr& indices ) {
    const auto N = v.size();

    if ( N == 0 ) {
        std::stringstream ss;
        ss << "Vector is empty. Cannot convert to pcl::PointIndices. ";
        throw( std::runtime_error( ss.str() ) );
    }

    indices->indices.resize(N);

    std::copy( v.begin(), v.end(), indices->indices.begin() );
}

}

#endif //POINTCLOUDUTILS_COMMON_HPP
