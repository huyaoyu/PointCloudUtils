//
// Created by yaoyu on 3/29/20.
//

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <PCCommon/IO.hpp>

#include "CameraGeometry/CameraPose2PCL.hpp"
#include "CameraGeometry/IO.hpp"
#include "PCCommon/IO.hpp"

using namespace std;

int main(int argc, char* argv []) {
    cout << "Hello, TryCSVIO! " << endl;

    cout << "argc = " << argc << endl;
    for ( int i = 0; i < argc; ++i ) {
        cout << "argv[" << i << "] = " << argv[i] << endl;
    }

    if ( argc == 1 ) {
        cout << "Not enough arguments. " << endl;
        return 1;
    }

    const string fn = argv[1];

    // Load a CSV file.
    Eigen::VectorXi id;
    Eigen::MatrixXf quat;
    Eigen::MatrixXf pos;

    read_camera_poses_csv(fn, id, quat, pos);

    // Show the sizes of the loaded values.
    cout << "id.rows() = " << id.rows() << ", id.cols() = " << id.cols() << ". " << endl;
    cout << "quat.rows() = " << quat.rows() << ", quat.cols() = " << quat.cols() << ". " << endl;
    cout << "pos.rows() = " << pos.rows() << ", pos.cols() = " << pos.cols() << ". " << endl;

    // Show the elements loaded.
    cout << "id(0) = " << id(0) << ", id(Eigen::last) = " << id(Eigen::last) << ". " << endl;
    cout << "quat(0, Eigen::all) = " << quat(0, Eigen::all) << ", "
         << "quat(Eigen::last, Eigen::all) = " << quat(Eigen::last, Eigen::all) << ". " << endl;
    cout << "pos(0, Eigen::all) = " << pos(0, Eigen::all) << ", "
         << "pos(Eigen::last, Eigen::all) = " << pos(Eigen::last, Eigen::all) << ". " << endl;

    // Convert the loaded camera poses to a pcl point cloud.
    pcl::PointCloud<pcl::PointNormal>::Ptr pNormal ( new pcl::PointCloud<pcl::PointNormal> );
    pcu::convert_camera_poses_2_pcl( quat, pos, pNormal );

    // Save the point cloud.
    pcu::write_point_cloud<pcl::PointNormal>( argv[2], pNormal );

    return 0;
}