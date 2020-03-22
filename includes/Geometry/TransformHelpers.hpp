//
// Created by yaoyu on 3/21/20.
//

#ifndef POINTCLOUDUTILS_TRANSFORMHELPERS_HPP
#define POINTCLOUDUTILS_TRANSFORMHELPERS_HPP

#include <iostream>

#include <Eigen/Dense>

template <typename realT>
void make_trans_mat_by_change_of_normalized_basis(
        const realT* b0, const realT* b1, const realT* b2,
        const realT* t, Eigen::Matrix4<realT>& mat ) {
    assert( b0 != nullptr );
    assert( b1 != nullptr );
    assert( b2 != nullptr );
    assert( t != nullptr );

    Eigen::Matrix3<realT> R;
    R << b0[0], b1[0], b2[0],
         b0[1], b1[1], b2[1],
         b0[2], b1[2], b2[2];
    Eigen::Vector3<realT> T;
    T << t[0], t[1], t[2];

    mat = Eigen::Matrix4<realT>::Identity();

    mat.block(0,0,3,3) = R.transpose();
    mat.block(0,3,3,1) = -R.transpose() * T;
}

template < typename realT >
void make_trans_mat_by_change_of_normalized_basis(
        const Eigen::Vector3<realT>& b0,
        const Eigen::Vector3<realT>& b1,
        const Eigen::Vector3<realT>& b2,
        const Eigen::Vector3<realT>& t,
        Eigen::Matrix4<realT>& mat ) {

    Eigen::Matrix3<realT> R;
    R.block(0,0,3,1) = b0;
    R.block(0,1,3,1) = b1;
    R.block(0,2,3,1) = b2;

    mat = Eigen::Matrix4<realT>::Identity();

    mat.block(0,0,3,3) = R.transpose();
    mat.block(0,3,3,1) = -R.transpose() * t;
}

#endif //POINTCLOUDUTILS_TRANSFORMHELPERS_HPP
