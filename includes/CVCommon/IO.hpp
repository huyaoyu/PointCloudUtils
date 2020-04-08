//
// Created by yaoyu on 4/7/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_CVCOMMON_IO_HPP
#define POINTCLOUDUTILS_INCLUDES_CVCOMMON_IO_HPP

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

void write_eigen_matrixXf_2_image( const std::string &fn, const Eigen::MatrixXf &mat,
        float minVal, float maxVal);

#endif //POINTCLOUDUTILS_INCLUDES_CVCOMMON_IO_HPP
