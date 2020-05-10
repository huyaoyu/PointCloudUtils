//
// Created by yaoyu on 4/8/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_PCCOMMON_BBOX_HPP
#define POINTCLOUDUTILS_INCLUDES_PCCOMMON_BBOX_HPP

#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>

namespace pcu
{
template < typename pT, typename rT >
struct OBB {
    pT minPoint;
    pT maxPoint;
    pT position;
    Eigen::Matrix3<rT> rotMat;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template < typename pT, typename rT >
void get_obb( const typename pcl::PointCloud<pT>::Ptr pInput,
                     pT &minPoint, pT &maxPoint,
                     pT &position, Eigen::Matrix3<rT> &rotMat ) {
    pcl::MomentOfInertiaEstimation<pT> extractor;
    extractor.setInputCloud(pInput);
    extractor.compute();

    Eigen::Matrix3f tempRotMat;

    extractor.getOBB( minPoint, maxPoint, position, tempRotMat );

    rotMat = tempRotMat.cast<rT>();
}

template < typename pT, typename rT >
void get_obb( const typename pcl::PointCloud<pT>::Ptr pInput,
        OBB<pT, rT> &obb ) {
    get_obb<pT, rT>( pInput, obb.minPoint, obb.maxPoint, obb.position, obb.rotMat );
}

}

#endif //POINTCLOUDUTILS_INCLUDES_PCCOMMON_BBOX_HPP
