//
// Created by yaoyu on 3/29/20.
//

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "CameraGeometry/CameraProjection.hpp"
#include "DataInterfaces/Plain/Matrix.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    cout << "Hello TryReadCameraMatrix! " << endl;

    cout << "argc = " << argc << endl;
    for ( int i = 0; i < argc; ++i ) {
        cout << "argv[" << i << "] = " << argv[i] << endl;
    }

    if ( argc == 1 ) {
        throw( std::runtime_error("Not enough arguments. ") );
    }

    Eigen::MatrixXf P1;

    read_matrix( argv[1], 3, 4, " ", P1 );

    cout << "P1 = " << endl;
    cout << P1 << endl;

    CameraProjection<float> cp;
    cp.K = P1.block(0, 0, 3, 3);

    cout << "cp.K = " << endl;
    cout << cp.K << endl;

    return 0;
}