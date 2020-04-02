//
// Created by yaoyu on 4/1/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_PCCOMMON_PROJECTION_HPP
#define POINTCLOUDUTILS_INCLUDES_PCCOMMON_PROJECTION_HPP

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>

namespace pcu
{

template < typename pT >
typename pcl::PointCloud<pT>::Ptr project_2_plane(
        const typename pcl::PointCloud<pT>::Ptr pInput,
        const pcl::PointNormal &pn ) {
    // The plane coefficients.
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients() );
    coefficients->values.resize(4);
    coefficients->values[0] = pn.normal_x;
    coefficients->values[1] = pn.normal_y;
    coefficients->values[2] = pn.normal_z;
    coefficients->values[3] = -(
            pn.x * pn.normal_x +
            pn.y * pn.normal_y +
            pn.z * pn.normal_z );

    // Filter.
    pcl::ProjectInliers<pT> proj;
    typename pcl::PointCloud<pT>::Ptr pProjected ( new pcl::PointCloud<pT> );
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(pInput);
    proj.setModelCoefficients(coefficients);
    proj.filter(*pProjected);

    return pProjected;
}

}

#endif //POINTCLOUDUTILS_INCLUDES_PCCOMMON_PROJECTION_HPP
