//
// Created by yaoyu on 4/4/20.
//

#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Dense>

#include <pcl/filters/crop_box.h>

#include "PCCommon/BBox.hpp"
//#include "PCCommon/common.hpp"
//#include "PCCommon/extraction.hpp"
#include "PCCommon/IO.hpp"
//#include "PCCommon/Transform.hpp"
#include "Profiling/SimpleTime.hpp"

#include "Tools/CropByOBBox.hpp"

int main( int argc, char* argv[] ) {
    QUICK_TIME_START(te)

    std::cout << "Hello, ShadowCrop! " << std::endl;

    if ( argc <= 3 ) {
        std::stringstream ss;
        ss << "Not enough arguments. ";
        throw( std::runtime_error( ss.str() ) );
    }

    std::string refFn = argv[1];
    std::string cropFn = argv[2];

    // Load the reference point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pRef =
            pcu::read_point_cloud<pcl::PointXYZ>( refFn );

    // Find the oriented bounding box.
    pcl::PointXYZ obbMinPoint, obbMaxPoint, obbPosition;
    Eigen::Matrix3f obbRotMat;

    pcu::get_obb<pcl::PointXYZ, float>(pRef, obbMinPoint, obbMaxPoint, obbPosition, obbRotMat);

    std::cout << "oobMinPoint = " << std::endl << obbMinPoint << std::endl;
    std::cout << "oobMaxPoint = " << std::endl << obbMaxPoint << std::endl;
    std::cout << "oobPosition = " << std::endl << obbPosition << std::endl;
    std::cout << "oobRotMat = " << std::endl << obbRotMat << std::endl;

    // Load the point cloud to be cropped.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pToBeCropped =
            pcu::read_point_cloud<pcl::PointXYZ>( cropFn );

//    // Transform the point cloud.
//    Eigen::Matrix4f transMat =
//            pcu::create_eigen_transform_matrix_1( obbPosition, obbRotMat );
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pTransformed =
//            pcu::transform_point_cloud<pcl::PointXYZ, float>( pToBeCropped, transMat );
//
////    pcl::PointCloud<pcl::PointXYZ>::Ptr pTransformed =
////            pcu::transform_point_cloud<pcl::PointXYZ, float>( pToBeCropped, obbPosition, obbRotMat );
//
//    // Crop the transformed point cloud by the bounding box.
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pTransCropped =
//            pcu::crop_by_CropBox<pcl::PointXYZ>(pTransformed, obbMinPoint, obbMaxPoint);
//
//    // The transform matrix for transforming the cropped point cloud back to the original frame.
////    Eigen::Matrix4f transMat =
////            pcu::create_eigen_transform_matrix_0( obbPosition, obbRotMat );
//
//    transMat = pcu::create_eigen_transform_matrix_0( obbPosition, obbRotMat );
//
//    // Transform back the point cloud.
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pCropped =
//            pcu::transform_point_cloud<pcl::PointXYZ, float>( pTransCropped, transMat );


    pcl::PointCloud<pcl::PointXYZ>::Ptr pCropped =
            pcu::crop_by_oriented_bbox<pcl::PointXYZ, float>( pToBeCropped,
                    obbMinPoint, obbMaxPoint, obbPosition, obbRotMat );

    // Write the point cloud.
    std::string outFn = argv[3];
    pcu::write_point_cloud<pcl::PointXYZ>(outFn, pCropped);

    QUICK_TIME_END(te)

    std::cout << "Shadow crop in " << te << " ms. " << std::endl;

    return 0;
}