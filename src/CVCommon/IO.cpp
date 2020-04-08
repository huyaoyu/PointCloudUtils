//
// Created by yaoyu on 4/7/20.
//

#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>

#include "CVCommon/IO.hpp"

void write_eigen_matrixXf_2_image( const std::string &fn, const Eigen::MatrixXf &mat,
                           float minVal, float maxVal) {
    assert( maxVal > minVal );

//    // Test use.
//    std::cout << "mat.minCoeff() = " << mat.minCoeff() << ", mat.maxCoeff() = " << mat.maxCoeff() << std::endl;

    Eigen::MatrixXf m = ( ( mat.array() - minVal ) / ( maxVal - minVal ) ).matrix() * 255.0f;

//    // Test use.
//    std::cout << "m.maxCoeff() = " << m.maxCoeff() << std::endl;

    // Convert the Eigen matrix to OpenCV object.
    cv::Mat_<float> imgF;
    cv::eigen2cv( m, imgF );

//    // Test use.
//    cv::Scalar_<float> avgF = cv::mean(imgF);
//    std::cout << "avgF = " << avgF << std::endl;
//    double minF, maxF;
//    cv::minMaxLoc(imgF, &minF, &maxF);
//    std::cout << "minF = " << minF << ", maxF = " << maxF << std::endl;

    // Convert the float image to CV_8UC1.
    cv::Mat img;
    imgF.convertTo(img, CV_8UC1);

//    // Test use.
//    cv::Scalar_<uint8_t> avgInt = cv::mean(img);
//    double minInt, maxInt;
//    cv::minMaxLoc( img, &minInt, &maxInt );
//    std::cout << "avgInt = " << avgInt << std::endl;
//    std::cout << "minInt = " << minInt << ", maxInt = " << maxInt << std::endl;

    // Write the image.
    std::vector<int> params({ cv::IMWRITE_PNG_COMPRESSION, 0 });
    cv::imwrite( fn, img, params );
}
