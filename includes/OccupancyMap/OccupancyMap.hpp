//
// Created by yaoyu on 5/6/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_OCCUPANCYMAP_OCCUPANCYMAP_HPP
#define POINTCLOUDUTILS_INCLUDES_OCCUPANCYMAP_OCCUPANCYMAP_HPP

#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

#include "Exception/Common.hpp"
#include "CameraGeometry/CameraProjection.hpp"

#include "OccupancyMapBuilder.hpp"

struct OccupancyMapException : virtual exception_common_base {};
struct CoordinateCheckFailed : virtual OccupancyMapException {};
struct AlreadyInitialized    : virtual OccupancyMapException {};

#define EXCEPTION_COOR_CHK_FAIL(point) \
    {\
        std::stringstream point##_ss; \
        point##_ss << "coordToKeyChecked() fails on " << point; \
        BOOST_THROW_EXCEPTION( CoordinateCheckFailed() << ExceptionInfoString(point##_ss.str()) ); \
    }

namespace pcu {

class OccupancyMap {
public:
    OccupancyMap();
    ~OccupancyMap();

    void initialize(float res);
    octomap::OcTree& get_octree();

    void set_basic_parameters( float res,
            float ocpThres, float probH, float probM, float clampMin, float clampMax );

    template < typename pT, typename rT >
    void build_from_pcl_and_cam_proj(const typename pcl::PointCloud<pT>::Ptr pInCloud,
                                     const std::vector<CameraProjection<rT> > &camProjs);

    void find_frontiers();
    template < typename rT >
    void get_coordinates_by_index(std::size_t ix, std::size_t iy, std::size_t iz,
                                  rT &x, rT &y, rT &z);
    void write_voxels_as_list(const std::string &fn);

    template < typename pT >
    void insert_point_cloud( typename pcl::PointCloud<pT>::Ptr pInput,
           const pcl::PointIndices::Ptr pIndices,
           const Eigen::Vector3f &sensorWP );

    template < typename pT >
    void insert_point_cloud( typename pcl::PointCloud<pT>::Ptr pInput,
                             const Eigen::Vector3f &sensorWP );

    void read(const std::string &fn);
    void write(const std::string &fn);

private:
    OccupancyMap( const OccupancyMap &other ) {}
    OccupancyMap( OccupancyMap &&other ) {}
    OccupancyMap& operator = ( const OccupancyMap &other ) { return *this; }
    OccupancyMap& operator = ( OccupancyMap &&other ) { return *this; }

private:
    void refresh_basic_parameters();

    template < typename pT >
    std::shared_ptr<octomap::Pointcloud> convert_pcl_2_pc(
            typename pcl::PointCloud<pT>::Ptr pInput );

    template < typename pT >
    std::shared_ptr<octomap::Pointcloud> extract_pc_from_pcl(
            typename pcl::PointCloud<pT>::Ptr pInput,
            const pcl::PointIndices::Ptr pIndices);

    void get_dense_grid_index( const double* minPoint,
            const octomap::point3d &point,
            std::size_t *index );

    void get_dense_grid_index( const octomap::OcTreeKey &key,
            std::size_t *index );

    void traverse_octree_and_fill_dense_grid( CMask *denseGrid );

private:
    std::shared_ptr<octomap::OcTree> pOcTree;

    float resolution;
    float occupancyThreshold;
    float probHit;
    float probMiss;
    float clampingThresholdMin;
    float clampingThresholdMax;

    octomap::OcTreeKey refKey;
    std::size_t vx;
    std::size_t vy;
    std::size_t vz;

    CR_DenseGrid denseGrid;
};

template < typename pT, typename rT >
void OccupancyMap::build_from_pcl_and_cam_proj(
        const typename pcl::PointCloud<pT>::Ptr pInCloud,
        const std::vector<CameraProjection<rT> > &camProjs) {
    OccupancyMapBuilder<rT> omb;
    omb.template build_occupancy_map<pT>(pInCloud, camProjs, *pOcTree);
}

template < typename pT >
std::shared_ptr<octomap::Pointcloud> OccupancyMap::convert_pcl_2_pc(
        typename pcl::PointCloud<pT>::Ptr pInput ) {
    const auto N = pInput->size();

    assert( N > 0 );

    auto pPC = std::make_shared<octomap::Pointcloud>();
    pPC->reserve( N );

    for ( std::size_t i = 0; i < N; ++i ) {
        pT &pclPoint = pInput->at( i );
        octomap::point3d point( pclPoint.x, pclPoint.y, pclPoint.z );
        pPC->push_back( point );
    }

    return pPC;
}

template < typename pT >
std::shared_ptr<octomap::Pointcloud> OccupancyMap::extract_pc_from_pcl(
        typename pcl::PointCloud<pT>::Ptr pInput,
        const pcl::PointIndices::Ptr pIndices) {
    const auto N = pIndices->indices.size();

    assert( N > 0 );

    auto pPC = std::make_shared<octomap::Pointcloud>();
    pPC->reserve( N );

    for ( std::size_t i = 0; i < N; ++i ) {
        pT &pclPoint = pInput->at( pIndices->indices[i] );
        octomap::point3d point( pclPoint.x, pclPoint.y, pclPoint.z );
        (*pPC)[i] = point;
    }

    return pPC;
}

template < typename pT >
void OccupancyMap::insert_point_cloud( typename pcl::PointCloud<pT>::Ptr pInput,
        const pcl::PointIndices::Ptr pIndices,
        const Eigen::Vector3f &sensorWP ) {
    // Convert the PCL PointCloud to OctoMap Pointcloud.
    std::shared_ptr<octomap::Pointcloud> pPC =
            extract_pc_from_pcl<pT>(pInput, pIndices);

    // Convert the Eigen vector3f to OctoMap point3d.
    octomap::point3d origin( sensorWP(0), sensorWP(1), sensorWP(2) );

    pOcTree->insertPointCloud( *pPC, origin );
}

template < typename pT >
void OccupancyMap::insert_point_cloud( typename pcl::PointCloud<pT>::Ptr pInput,
                                       const Eigen::Vector3f &sensorWP ) {
    // Convert the PCL PointCloud to OctoMap Pointcloud.
    std::shared_ptr<octomap::Pointcloud> pPC =
            convert_pcl_2_pc<pT>(pInput);

    // Convert the Eigen vector3f to OctoMap point3d.
    octomap::point3d origin( sensorWP(0), sensorWP(1), sensorWP(2) );

    pOcTree->insertPointCloud( *pPC, origin );
}

}

#endif //POINTCLOUDUTILS_INCLUDES_OCCUPANCYMAP_OCCUPANCYMAP_HPP
