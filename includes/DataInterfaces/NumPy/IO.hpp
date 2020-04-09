//
// Created by yaoyu on 4/8/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_DATAINTERFACES_NUMPY_IO_HPP
#define POINTCLOUDUTILS_INCLUDES_DATAINTERFACES_NUMPY_IO_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <cnpy.h>
#include <Eigen/Dense>

template < typename rT >
void write_depth_map_2_npy( const std::string &fn,
        const Eigen::MatrixX<rT> &mat,
        rT minLimit = 0 ) {
    const std::size_t rows = mat.rows();
    const std::size_t cols = mat.cols();
    std::vector<rT> data;

    for ( std::size_t j = 0; j < cols; ++j ) {
        for ( std::size_t i = 0; i < rows; ++i ) {
            const rT value = mat( i, j );

            if ( value > minLimit ) {
                data.push_back(static_cast<rT>(j));
                data.push_back(static_cast<rT>(i));
                data.push_back(value);
            }
        }
    }

    const std::size_t N = data.size();

    if ( 0 == N ) {
        std::stringstream ss;
        ss << "No pixels have depth over the limit of " << minLimit;
        throw( std::runtime_error( ss.str() ) );
    }

    cnpy::npy_save( fn, &data[0], { N/3, 3 }, "w" );
}

template < typename rT >
void write_eigen_matrix_2_npy( const std::string &fn,
        const Eigen::MatrixX<rT> &mat ) {
    const std::size_t rows = mat.rows();
    const std::size_t cols = mat.cols();

    if ( mat.IsRowMajor ) {
        cnpy::npy_save( fn, mat.data(), { rows, cols }, "w" );
    } else {
        Eigen::MatrixX<rT> temp = mat.transpose();
        cnpy::npy_save( fn, temp.data(), { rows, cols }, "w" );
    }
}

#endif //POINTCLOUDUTILS_INCLUDES_DATAINTERFACES_NUMPY_IO_HPP
