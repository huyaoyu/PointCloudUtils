//
// Created by yaoyu on 3/29/20.
//

#ifndef POINTCLOUDUTILS_CAMERA_GEOMETRY_IO_HPP
#define POINTCLOUDUTILS_CAMERA_GEOMETRY_IO_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "DataInterfaces/CSV/csv.h"
#include "Profiling/SimpleTime.hpp"

/**
 * Read the camera poses stored in a CSV file.
 * The CSV file have 8 columns. The definitions of the columns are:
 * id, qw, qx, qy, qz, x, y, z.
 * There is not a header in the CSV file.
 * The column id contains ids as integers.
 *
 * @param fn The file name.
 * @param id A Eigen::VectorX contains the integer ids.
 * @param quat A Eigen::MatrixX contains the quaternion data.
 * @param pos A Eigen::MatrixX contains the position data.
 */
template < typename rT >
void read_camera_poses_csv(const std::string& fn,
        Eigen::VectorXi& id, Eigen::MatrixX<rT>& quat, Eigen::MatrixX<rT>& pos) {
    QUICK_TIME_START(te)

    // A third-party CSV parsing implementation.
    io::CSVReader<8> csvIn(fn);
    csvIn.set_header( "id",
            "qw", "qx", "qy", "qz", "x", "y", "z" );

    // The temporary variables for parsing different columns.
    int cid;
    rT qw, qx, qy, qz;
    rT x, y, z;

    // Temporary storage.
    std::vector<int> vId;
    std::vector<rT> vQ;
    std::vector<rT> vP;

    // Parse the CSV file.
    while( csvIn.read_row( cid, qw, qx, qy, qz, x, y, z ) ) {
        vId.push_back(cid);
        vQ.push_back(qw); vQ.push_back(qx); vQ.push_back(qy); vQ.push_back(qz);
        vP.push_back(x); vP.push_back(y); vP.push_back(z);
    }

    // Convert the vectors to Eigen objects.
    const int N = static_cast<int>( vId.size() );
    id.resize(N);
    quat.resize(N, 4);
    pos.resize(N, 3);

    // Eigen actually copies the data with operator=.
    id   = Eigen::Map<Eigen::VectorXi>( vId.data(), N );
    quat = Eigen::Map<Eigen::Matrix<rT, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>( vQ.data(), N, 4 );
    pos  = Eigen::Map<Eigen::Matrix<rT, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>( vP.data(), N, 3 );

    QUICK_TIME_END(te)

    std::cout << "Read camera poses from CSV file in " << te << " ms. " << std::endl;
}

#endif //POINTCLOUDUTILS_CAMERA_GEOMETRY_IO_HPP
