//
// Created by yaoyu on 3/21/20.
//

#include <iostream>

#include <Eigen/Dense>

#include "Geometry/TransformHelpers.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    cout << "Hello, TryTransformHelpers." << endl;

    typedef float real;

    // The new normalized base vectors.
    real b0[3] = { 0, -1, 0 };
    real b1[3] = { 1,  0, 0 };
    real b2[3] = { 0,  0, 1 };

    // The translation measurement.
    real t[3] = {2, -2, 0};

    // The transform matrix.
    Eigen::Matrix4<real> mat;

    make_trans_mat_by_change_of_normalized_basis(
            b0, b1, b2, t, mat);

    cout << "mat = " << endl << mat << endl;

    // Use eigen vectors.

    Eigen::Vector3<real> eb0, eb1, eb2;
    eb0 << 0, -1, 0;
    eb1 << 1, 0, 0;
    eb2 << 0, 0, 1;

    Eigen::Vector3<real> et;
    et << 2, -2, 0;

    Eigen::Matrix4<real> emat;

    make_trans_mat_by_change_of_normalized_basis(
            eb0, eb1, eb2, et, emat );

    cout << "emat = " << endl << emat << endl;

    return 0;
}