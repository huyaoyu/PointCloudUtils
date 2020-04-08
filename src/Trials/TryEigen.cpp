//
// Created by yaoyu on 4/4/20.
//

#include <iostream>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#define SHOW_MATRIX(x) \
    std::cout << #x << " = " << std::endl << x << std::endl;

int main( int argc, char* argv[] ) {
    std::cout << "Hello, TryEigen! " << std::endl;

    Eigen::Matrix4f transMat = Eigen::Matrix4f::Identity();

    SHOW_MATRIX(transMat)

    const auto pi = boost::math::constants::pi<float>();
    const auto angleAxis = Eigen::AngleAxisf( 0.5f * pi, Eigen::Vector3f::UnitZ() );
    Eigen::Quaternionf quat = Eigen::Quaternionf( angleAxis );
    Eigen::Matrix3f rotMat = quat.toRotationMatrix();

    SHOW_MATRIX(rotMat)

    Eigen::Vector4f translation({ 1.0f, 2.0f, 3.0f, 1.0f });

    SHOW_MATRIX(translation)

    transMat.block(0,0,3,3) = rotMat;
//    transMat( Eigen::seq(0,2), Eigen::seq(0,2) ) = rotMat; // Also works.
    transMat.col(3) = translation;

    SHOW_MATRIX(transMat)

    Eigen::MatrixXf pointMat = Eigen::MatrixXf::Random(3, 5);
    SHOW_MATRIX(pointMat)

    Eigen::MatrixXf transPoint = pointMat.colwise() - Eigen::Vector3f::Ones();
    SHOW_MATRIX(transPoint)

    return 0;
}