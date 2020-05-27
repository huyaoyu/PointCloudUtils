//
// Created by yaoyu on 3/27/20.
//

#ifndef POINTCLOUDUTILS_PCCOMMON_IO_HPP
#define POINTCLOUDUTILS_PCCOMMON_IO_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "common.hpp"
#include "Profiling/SimpleTime.hpp"

namespace pcu {

template<typename T>
typename pcl::PointCloud<T>::Ptr read_point_cloud(const std::string &fn) {
    // ========== Read the point cloud from the file. ==========
    std::cout << "Loading points from " << fn << " ... \n";

    QUICK_TIME_START(teReadPointCloud);

    typename pcl::PointCloud<T>::Ptr pOutCloud ( new pcl::PointCloud<T> );

    if (pcl::io::loadPLYFile<T>(fn, *pOutCloud) == -1) {
        std::stringstream ss;
        ss << "Failed to read: " << fn;
        throw (std::runtime_error(ss.str()));
    }
    QUICK_TIME_END(teReadPointCloud);

    std::cout << pOutCloud->size() << " points loaded in " << teReadPointCloud << "ms. \n";

    return pOutCloud;
}

template<typename T>
void read_point_cloud(const std::string &fn, typename pcl::PointCloud<T>::Ptr &pOutCloud) {
    // ========== Read the point cloud from the file. ==========
    std::cout << "Loading points from " << fn << " ... \n";

    QUICK_TIME_START(teReadPointCloud);
    if (pcl::io::loadPLYFile<T>(fn, *pOutCloud) == -1) {
        std::stringstream ss;
        ss << "Failed to read: " << fn;
        throw (std::runtime_error(ss.str()));
    }
    QUICK_TIME_END(teReadPointCloud);

    std::cout << pOutCloud->size() << " points loaded in " << teReadPointCloud << "ms. \n";
}

template < typename pT, typename rT >
void read_point_cloud_xyz_as_eigen_matrix( const std::string &fn,
        Eigen::MatrixX<rT> &mat ) {
    typename pcl::PointCloud<pT>::Ptr pCloud = read_point_cloud<pT>(fn);
    convert_pcl_2_eigen_matrix<pT, rT>(pCloud, mat);
}

template < typename pT >
void write_point_cloud( const std::string& fn, const typename pcl::PointCloud<pT>::Ptr& pOutput, bool flagBinary=true ) {
    QUICK_TIME_START(te)

    pcl::PLYWriter writer;
    std::cout << "Saving the filtered point cloud. \n";
    writer.write(fn, *pOutput, flagBinary, false);

    QUICK_TIME_END(te)

    std::cout << "Save the point cloud to " << fn << " in " << te << " ms. \n";
}

} // namespace pcu.
#endif //POINTCLOUDUTILS_PCCOMMON_IO_HPP
