//
// Created by yaoyu on 3/25/20.
//

#ifndef POINTCLOUDUTILS_EXTRACTION_HPP
#define POINTCLOUDUTILS_EXTRACTION_HPP

#include "PCCommon/common.hpp"

#include <pcl/filters/extract_indices.h>

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
    extract.filter( *pOutput );
}

template < typename pT, typename iT >
void extract_points( const typename pcl::PointCloud<pT>::Ptr pInput,
                            typename pcl::PointCloud<pT>::Ptr pOutput,
                            const std::vector<iT>& indices ) {
    pcl::PointIndices::Ptr pclIndices = convert_vector_2_pcl_indices( indices );

    extract_points<pT>( pInput, pOutput, pclIndices );
}

} // namespace pcu.

#endif //POINTCLOUDUTILS_EXTRACTION_HPP
