//
// Created by yaoyu on 3/25/20.
//

#ifndef POINTCLOUDUTILS_EXTRACTION_HPP
#define POINTCLOUDUTILS_EXTRACTION_HPP

#include "PCCommon/common.hpp"

#include <pcl/filters/extract_indices.h>

namespace pcu
{

template < typename pT, typename iT >
static void extract_points( const typename pcl::PointCloud<pT>::Ptr& pInput,
                            typename pcl::PointCloud<pT>::Ptr& pOutput,
                            const std::vector<iT>& indices) {
    pcl::PointIndices::Ptr pclIndices (new pcl::PointIndices() );
    pclIndices->indices.resize(indices.size() );
    std::copy(indices.begin(), indices.end(), pclIndices->indices.begin() );

    pcl::ExtractIndices<pT> extract;
    extract.setInputCloud(pInput);
    extract.setIndices(pclIndices);
    extract.setNegative(false);
    extract.filter( *pOutput );
}

} // namespace pcu.

#endif //POINTCLOUDUTILS_EXTRACTION_HPP
