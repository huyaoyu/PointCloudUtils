//
// Created by yaoyu on 7/1/20.
//

#ifndef POINTCLOUDUTILS_NORMALHELPERS_HPP
#define POINTCLOUDUTILS_NORMALHELPERS_HPP

#include <iostream>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include "Profiling/SimpleTime.hpp"

namespace pcu
{

/**
* Flip the normal stored in a pcl::PointNormal typed point cloud.
*
* @param pInput The input point cloud.
* @param vx The x coordinate of the view point.
* @param vy The y coordinate of the view point.
* @param vz The z coordinate of the view point.
*/
template < typename PT >
void flip_normal_inplace( typename pcl::PointCloud<PT>::Ptr& pInput,
        const float vx, const float vy, const float vz, bool flagFlipOpposite=false ) {
    QUICK_TIME_START(te)

    std::cout << "Start flipping the normal. " << std::endl;

    // Loop over all the points in the point cloud.
    for ( auto iter = pInput->begin(); iter != pInput->end(); iter++ ) {
        if ( pcl::isFinite(*iter) ) {
            pcl::flipNormalTowardsViewpoint( *iter, vx, vy, vz,
                    (*iter).normal_x, (*iter).normal_y, (*iter).normal_z );
        }
    }

    if ( flagFlipOpposite ) {
        for ( auto iter = pInput->begin(); iter != pInput->end(); iter++ ) {
            if ( pcl::isFinite(*iter) ) {
                (*iter).normal_x *= -1;
                (*iter).normal_y *= -1;
                (*iter).normal_z *= -1;
            }
        }
    }

    QUICK_TIME_END(te)
    std::cout << "Execute flip_normal_inplace() in " << te << "ms. " << std::endl;
}

/**
* Flip the normal stored in a pcl::PointNormal typed point cloud.
*
* @param pInput The input point cloud.
* @param vp The viewing point, with the last element being 1.
*/
template < typename PT >
void flip_normal_inplace( typename pcl::PointCloud<PT>::Ptr& pInput,
                                 const Eigen::Vector4f& vp, bool flagFlipOpposite=false ) {
    flip_normal_inplace<PT>(pInput, vp(0), vp(1), vp(2), flagFlipOpposite);
}

}

#endif //POINTCLOUDUTILS_NORMALHELPERS_HPP
