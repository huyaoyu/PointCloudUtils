//
// Created by yaoyu on 3/29/20.
//

#ifndef POINTCLOUDUTILS_CAMERAPOSE2PCL_HPP
#define POINTCLOUDUTILS_CAMERAPOSE2PCL_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "CameraGeometry/IO.hpp"
#include "PCCommon/IO.hpp"
#include "Profiling/SimpleTime.hpp"

namespace pcu
{

template < typename rT >
void convert_camera_poses_2_pcl(
        const Eigen::MatrixX<rT>& quat, const Eigen::MatrixX<rT>& pos,
        typename pcl::PointCloud<pcl::PointNormal>::Ptr pOutput ) {
    QUICK_TIME_START(te)

    const int N = quat.rows();
    assert( N == pos.rows() );

    pOutput->resize( N );

    const Eigen::Vector3<rT> zAxis(0.0f, 0.0f, 1.0f);

    for ( int i = 0; i < N; ++i ) {
        pcl::PointNormal point;

        point.x = pos(i, 0);
        point.y = pos(i, 1);
        point.z = pos(i, 2);

        // Make a Eigen quaternion.
        Eigen::Quaternion<rT> q(
                quat(i, 0), quat(i, 1), quat(i, 2), quat(i, 3) );

        // Convert the quaternion description into a normal vector.
        Eigen::Vector3<rT> normal = q * zAxis;

        // Update the normal point.
        point.normal_x = normal(0);
        point.normal_y = normal(1);
        point.normal_z = normal(2);

        // Dummy curvature.
        point.curvature = 0.0f;

        // Save.
        pOutput->at(i) = point;
    }

    QUICK_TIME_END(te)

    std::cout << "Convert camera poses to PCL point cloud in " << te << " ms. " << std::endl;
}

template < typename rT >
pcl::PointCloud<pcl::PointNormal>::Ptr convert_camera_poses_2_pcl(
        const Eigen::MatrixX<rT>& quat, const Eigen::MatrixX<rT>& pos ) {
    pcl::PointCloud<pcl::PointNormal>::Ptr pOutput ( new pcl::PointCloud<pcl::PointNormal> );

    convert_camera_poses_2_pcl( quat, pos, pOutput );

    return pOutput;
}

}

#endif //POINTCLOUDUTILS_CAMERAPOSE2PCL_HPP
