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

#include "FrontierMap.hpp"
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
    typedef f_map::FrontierOcTree Tree_t;
public:
    OccupancyMap();
    ~OccupancyMap();

    void initialize(float res);
//    octomap::OcTree& get_octree();
    f_map::FrontierOcTree& get_octree();

    void set_basic_parameters( float res,
            float ocpThres, float probH, float probM, float clampMin, float clampMax );

    template < typename pT, typename rT >
    void build_from_pcl_and_cam_proj(const typename pcl::PointCloud<pT>::Ptr pInCloud,
                                     const std::vector<CameraProjection<rT> > &camProjs);

    template < typename pT >
    void update_occupancy_by_pcl( const typename pcl::PointCloud<pT>::Ptr pInCloud);

    void find_frontiers();
    std::size_t num_frontiers() const { return nFrontiers; };
    bool check_frontier( float x, float y, float z ) const;

    /**
     * Check all the 27 neighboring voxels including the center one for
     * frontiers.
     *
     * @param x X-coordinate.
     * @param y Y-coordinate.
     * @param z Z-coordinate.
     * @param t If there are more than or equal to t neighbors being frontiers, return true;
     * @return true if near a frontier, false otherwise.
     */
    bool check_near_frontier( float x, float y, float z, int t=2 ) const;

    template < typename rT >
    void get_coordinates_by_index(std::size_t ix, std::size_t iy, std::size_t iz,
                                  rT &x, rT &y, rT &z) const;
    void write_voxels_as_list(const std::string &fn) const;

    template < typename pT >
    void insert_point_cloud( typename pcl::PointCloud<pT>::Ptr pInput,
           const pcl::PointIndices::Ptr pIndices,
           const Eigen::Vector3f &sensorWP );

    template < typename pT >
    void insert_point_cloud( typename pcl::PointCloud<pT>::Ptr pInput,
                             const Eigen::Vector3f &sensorWP );

    void read(const std::string &fn);
    void write(const std::string &fn) const;

private:
    OccupancyMap( const OccupancyMap &other ) {}
    OccupancyMap( OccupancyMap &&other ) {}
    OccupancyMap& operator = ( const OccupancyMap &other ) { return *this; }
    OccupancyMap& operator = ( OccupancyMap &&other ) { return *this; }

private:
    void refresh_basic_parameters();

    template < typename pT >
    std::shared_ptr<octomap::Pointcloud> convert_pcl_2_pc(
            typename pcl::PointCloud<pT>::Ptr pInput ) const;

    template < typename pT >
    std::shared_ptr<octomap::Pointcloud> extract_pc_from_pcl(
            typename pcl::PointCloud<pT>::Ptr pInput,
            const pcl::PointIndices::Ptr pIndices) const;

    void get_dense_grid_index( const double* minPoint,
            const octomap::point3d &point,
            std::size_t *index ) const;

    void get_dense_grid_index( const octomap::OcTreeKey &key,
            std::size_t *index ) const;

    void traverse_octree_and_fill_dense_grid( CMask *denseGrid );
    void traverse_octree_and_update_frontiers( CMask *denseGrid );

    void count_frontiers();

private:
    static const std::vector<std::vector<int>> neighborShift;

private:
//    std::shared_ptr<octomap::OcTree> pOcTree;
    std::unique_ptr<Tree_t> pFrontierOcTree;

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

    std::size_t nFrontiers;
};

template < typename pT, typename rT >
void OccupancyMap::build_from_pcl_and_cam_proj(
        const typename pcl::PointCloud<pT>::Ptr pInCloud,
        const std::vector<CameraProjection<rT> > &camProjs) {
    OccupancyMapBuilder<rT> omb;
    omb.template build_occupancy_map<pT, f_map::FrontierOcTreeNode>(pInCloud, camProjs, pFrontierOcTree.get());
}

template < typename pT >
void OccupancyMap::update_occupancy_by_pcl( const typename pcl::PointCloud<pT>::Ptr pInCloud) {
    const int N = pInCloud->size();
    for ( int i = 0; i < N; ++i ) {
        const auto& pclPoint = pInCloud->at(i);
        pFrontierOcTree->updateNode( pclPoint.x, pclPoint.y, pclPoint.z, true, true );
    }

    pFrontierOcTree->updateInnerOccupancy();
}

template < typename pT >
std::shared_ptr<octomap::Pointcloud> OccupancyMap::convert_pcl_2_pc(
        typename pcl::PointCloud<pT>::Ptr pInput ) const {
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
        const pcl::PointIndices::Ptr pIndices) const {
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

    pFrontierOcTree->insertPointCloud( *pPC, origin );
}

template < typename pT >
void OccupancyMap::insert_point_cloud( typename pcl::PointCloud<pT>::Ptr pInput,
                                       const Eigen::Vector3f &sensorWP ) {
    // Convert the PCL PointCloud to OctoMap Pointcloud.
    std::shared_ptr<octomap::Pointcloud> pPC =
            convert_pcl_2_pc<pT>(pInput);

    // Convert the Eigen vector3f to OctoMap point3d.
    octomap::point3d origin( sensorWP(0), sensorWP(1), sensorWP(2) );

    pFrontierOcTree->insertPointCloud( *pPC, origin );
}

inline bool OccupancyMap::check_frontier( float x, float y, float z ) const {
    const Tree_t::NodeType *node = pFrontierOcTree->search( x, y, z, 16 );

    if ( node ) {
        if ( node->is_frontier() ) {
            return true;
        }
    }

    return false;
}

inline bool OccupancyMap::check_near_frontier( float x, float y, float z, int t ) const {
    const auto key = pFrontierOcTree->coordToKey( x, y, z );
    int count = 0; // Number of frontier neighbors.
    bool flagFrontier = false;

    for ( const auto& shift : neighborShift ) {
        auto nk = key;
        nk[0] += shift[0];
        nk[1] += shift[1];
        nk[2] += shift[2];

        const Tree_t::NodeType *node = pFrontierOcTree->search( nk, 16 );

        if ( node ) {
            if ( node->is_frontier() ) {
                count++;

                if ( count >= t ) {
                    flagFrontier = true;
                    break;
                }
            }
        }
    }

    return flagFrontier;
}

}

#endif //POINTCLOUDUTILS_INCLUDES_OCCUPANCYMAP_OCCUPANCYMAP_HPP
