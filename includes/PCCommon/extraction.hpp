//
// Created by yaoyu on 3/25/20.
//

#ifndef POINTCLOUDUTILS_EXTRACTION_HPP
#define POINTCLOUDUTILS_EXTRACTION_HPP

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "PCCommon/common.hpp"
#include "Profiling/SimpleTime.hpp"

namespace pcu
{

template < typename pT >
void extract_points( const typename pcl::PointCloud<pT>::Ptr pInput,
                     typename pcl::PointCloud<pT>::Ptr pOutput,
                     const pcl::PointIndices::Ptr indices ) {
    pcl::ExtractIndices<pT> extract;
    extract.setInputCloud(pInput);
    extract.setIndices(indices);
    extract.setNegative(false);

    if ( pInput.get() == pOutput.get() ) {
        typename pcl::PointCloud<pT>::Ptr pTemp ( new pcl::PointCloud<pT> );
        extract.filter( *pTemp );
        pOutput = pTemp; // Should work.
    } else {
        extract.filter( *pOutput );
    }
}

template < typename pT >
typename pcl::PointCloud<pT>::Ptr extract_points( const typename pcl::PointCloud<pT>::Ptr pInput,
                     const pcl::PointIndices::Ptr indices ) {
    pcl::ExtractIndices<pT> extract;
    extract.setInputCloud(pInput);
    extract.setIndices(indices);
    extract.setNegative(false);

    typename pcl::PointCloud<pT>::Ptr pOutput( new pcl::PointCloud<pT> );
    extract.filter( *pOutput );
    return pOutput;
}

template < typename pT, typename iT >
typename pcl::PointCloud<pT>::Ptr extract_points( const typename pcl::PointCloud<pT>::Ptr pInput,
                     const std::vector<iT>& indices ) {
    typename pcl::PointCloud<pT>::Ptr pOutput ( new pcl::PointCloud<pT> );

    pcl::PointIndices::Ptr pclIndices = convert_vector_2_pcl_indices( indices );

    extract_points<pT>( pInput, pOutput, pclIndices );

    return pOutput;
}

template < typename pT, typename iT >
void extract_points( const typename pcl::PointCloud<pT>::Ptr pInput,
                            typename pcl::PointCloud<pT>::Ptr pOutput,
                            const std::vector<iT>& indices ) {
    pcl::PointIndices::Ptr pclIndices = convert_vector_2_pcl_indices( indices );

    extract_points<pT>( pInput, pOutput, pclIndices );
}

template <typename T>
typename pcl::PointCloud<T>::Ptr crop_by_CropBox(
        const typename pcl::PointCloud<T>::Ptr inCloud,
        const pcl::PointXYZ &minPoint,
        const pcl::PointXYZ &maxPoint )
{
    QUICK_TIME_START(te)

    Eigen::Vector4f p0 = pcu::create_eigen_vector4_from_xyz<T, float>(minPoint);
    Eigen::Vector4f p1 = pcu::create_eigen_vector4_from_xyz<T, float>(maxPoint);

    typename pcl::PointCloud<T>::Ptr pOutCloud ( new pcl::PointCloud<T> );

    pcl::CropBox<T> pass;
    pass.setMin( p0 );
    pass.setMax( p1 );
    pass.setInputCloud(inCloud);
    pass.filter(*pOutCloud);

    QUICK_TIME_END(te)

    std::cout << "Cropped " << pOutCloud->size() << " points. " << std::endl;
    std::cout << "Crop in " << te << "ms. " << std::endl;

    return pOutCloud;
}

} // namespace pcu.

#endif //POINTCLOUDUTILS_EXTRACTION_HPP
