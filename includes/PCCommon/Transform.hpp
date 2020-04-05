//
// Created by yaoyu on 4/4/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_PCCOMMON_TRANSFORM_HPP
#define POINTCLOUDUTILS_INCLUDES_PCCOMMON_TRANSFORM_HPP

#include <iostream>
#include <sstream>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>

#include "PCCommon/common.hpp"

namespace pcu
{

template < typename pT, typename rT >
typename pcl::PointCloud<pT>::Ptr transform_point_cloud(
        const typename pcl::PointCloud<pT>::Ptr pInput,
        const Eigen::Matrix4<rT> &mat ) {
    typename  pcl::PointCloud<pT>::Ptr transformed ( new pcl::PointCloud<pT> );
    pcl::transformPointCloud( *pInput, *transformed, mat );
    return transformed;
}

template < typename pT, typename rT >
typename pcl::PointCloud<pT>::Ptr transform_point_cloud(
        const typename pcl::PointCloud<pT>::Ptr pInput,
        const pcl::PointXYZ &position,
        const Eigen::Matrix3<rT> &rotMat ) {
    Eigen::Matrix4<rT> transMat =
            pcu::create_eigen_transform_matrix_1( position, rotMat );

    return transform_point_cloud<pT, rT>( pInput, transMat );
}

}

#endif //POINTCLOUDUTILS_INCLUDES_PCCOMMON_TRANSFORM_HPP
