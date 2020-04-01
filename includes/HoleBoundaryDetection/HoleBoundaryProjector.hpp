//
// Created by yaoyu on 3/31/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_HOLEBOUNDARYPROJECTION_HPP
#define POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_HOLEBOUNDARYPROJECTION_HPP

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

#include "CameraGeometry/CameraProjection.hpp"
#include "Profiling/SimpleTime.hpp"

namespace pcu
{

template < typename rT >
class HoleBoundaryPoints {
public:
    HoleBoundaryPoints(): cameraID(INVALID_CAMERA_ID) {}
    ~HoleBoundaryPoints() = default;

public:
    static const int INVALID_CAMERA_ID;

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;
    int cameraID;
    Eigen::Matrix3<rT> cameraR;
    Eigen::Vector3<rT> cameraT;
};

template <typename rT>
const int HoleBoundaryPoints<rT>::INVALID_CAMERA_ID = -1;

template < typename pT, typename rT >
class HoleBoundaryProjector {
public:
    typedef pcl::PointCloud<pT> PC_t;
    typedef pcl::PointCloud<pcl::PointNormal> PN_t;
    typedef std::vector< std::vector<int> > DS_t; // Disjoint set type.
    typedef std::shared_ptr<DS_t> DSPtr;

public:
    HoleBoundaryProjector() = default;
    ~HoleBoundaryProjector() = default;

    void process( const typename PC_t::Ptr pInput,
            const DSPtr pDS,
            const PN_t::Ptr pEquivalentNormals,
            const std::vector< CameraProjection<rT> >& cameraProjections,
            std::vector< HoleBoundaryPoints<rT> >& hbp);
};

template < typename pT, typename rT >
void HoleBoundaryProjector<pT, rT>::process(const typename PC_t::Ptr pInput,
        const DSPtr pDS,
        const PN_t::Ptr pEquivalentNormals,
        const std::vector< CameraProjection<rT> >& cameraProjections,
        std::vector<HoleBoundaryPoints<rT>> &hbp) {

}

}

#endif //POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_HOLEBOUNDARYPROJECTION_HPP
