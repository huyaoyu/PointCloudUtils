//
// Created by yaoyu on 4/8/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_TOOLS_CROPBYOBBOX_HPP
#define POINTCLOUDUTILS_INCLUDES_TOOLS_CROPBYOBBOX_HPP

#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Dense>

#include <pcl/filters/crop_box.h>

#include "PCCommon/common.hpp"
#include "PCCommon/extraction.hpp"
#include "PCCommon/Transform.hpp"

namespace pcu {

template<typename pT, typename rT>
typename pcl::PointCloud<pT>::Ptr crop_by_oriented_bbox(
        const typename pcl::PointCloud<pT>::Ptr pInput,
        const pcl::PointXYZ &obbMinPoint,
        const pcl::PointXYZ &obbMaxPoint,
        const pcl::PointXYZ &obbPosition,
        const Eigen::Matrix3<rT> &obbRotMat) {
    typedef typename pcl::PointCloud<pT>::Ptr pcPtr;

    // Transform the point cloud.
    Eigen::Matrix4<rT> transMat =
            create_eigen_transform_matrix_1( obbPosition, obbRotMat );

    pcPtr pTransformed = transform_point_cloud<pT, rT>( pInput, transMat );

    // Crop the transformed point cloud by the bounding box.
    pcPtr pTransCropped = crop_by_CropBox<pT>(pTransformed, obbMinPoint, obbMaxPoint);

    // New transformation matrix.
    transMat = create_eigen_transform_matrix_0( obbPosition, obbRotMat );

    // Transform back the point cloud.
    pcPtr pCropped = transform_point_cloud<pT, rT>( pTransCropped, transMat );

    return pCropped;
}

} // Namespace pcu.
#endif //POINTCLOUDUTILS_INCLUDES_TOOLS_CROPBYOBBOX_HPP
