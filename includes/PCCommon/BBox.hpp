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

template < typename pT >
void get_obb( const typename pcl::PointCloud<pT>::Ptr pInput,
                     pcl::PointXYZ &minPoint, pcl::PointXYZ &maxPoint,
                     pcl::PointXYZ &position, Eigen::Matrix3f &rotMat ) {
    pcl::MomentOfInertiaEstimation<pT> extractor;
    extractor.setInputCloud(pInput);
    extractor.compute();

    extractor.getOBB( minPoint, maxPoint, position, rotMat );
}

}

#endif //POINTCLOUDUTILS_INCLUDES_PCCOMMON_BBOX_HPP
