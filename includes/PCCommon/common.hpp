//
// Created by yaoyu on 3/25/20.
//

#ifndef POINTCLOUDUTILS_COMMON_HPP
#define POINTCLOUDUTILS_COMMON_HPP

#include <algorithm>    // std::sort
#include <cmath>
#include <iostream>
#include <numeric>      // std::iota
#include <sstream>
#include <string>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include <pcl/point_types.h>

namespace pcu
{

template < typename pT0, typename pT1 >
float distance_two_points( const pT0& p0, const pT1& p1 ) {
    const float d0 = p0.x - p1.x;
    const float d1 = p0.y - p1.y;
    const float d2 = p0.z - p1.z;

    return std::sqrt( d0 * d0 + d1 * d1 + d2 * d2 );
}

}

#endif //POINTCLOUDUTILS_COMMON_HPP
