//
// Created by yaoyu on 4/13/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_DATAINTERFACES_PLAIN_FROMVECTOR_HPP
#define POINTCLOUDUTILS_INCLUDES_DATAINTERFACES_PLAIN_FROMVECTOR_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>


template < typename rT, typename derived >
void convert_vector_2_eigen_vector( const std::vector<rT> &v,
        Eigen::MatrixBase<derived> &ev ) {
    const std::size_t N = v.size();
    assert(N > 0);

    for ( std::size_t i = 0; i < N; ++i ) {
        ev(i) = v[i];
    }
}

template < typename rT >
void convert_vector_2_eigen_mat3( const std::vector<rT> &v,
        Eigen::Matrix3<rT> &mat ) {
    mat << v[0], v[1], v[2],
           v[3], v[4], v[5],
           v[6], v[7], v[8];
}

#endif //POINTCLOUDUTILS_INCLUDES_DATAINTERFACES_PLAIN_FROMVECTOR_HPP
