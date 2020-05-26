//
// Created by yaoyu on 5/10/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_OCCUPANCYMAPBUILDER_HPP
#define POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_OCCUPANCYMAPBUILDER_HPP

#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <octomap/OcTree.h>

#include "CameraGeometry/CameraProjection.hpp"
#include "Exception/Common.hpp"
#include "Profiling/SimpleTime.hpp"

#include "CUDA/OccupancyMapBuilderRoutines.h"

namespace pcu
{

template < typename rT >
class OccupancyMapBuilder {
public:
    OccupancyMapBuilder():
        occThreshold(0.05), knnK(18), knnOcc(knnK/3) {}
    ~OccupancyMapBuilder() = default;

    void set_occlusion_threshold(rT t);
    void set_knn_k_and_knn_occ(int k, int occ);

    template < typename pT, typename nT >
    void build_occupancy_map(const typename pcl::PointCloud<pT>::Ptr pInCloud,
                             const std::vector< CameraProjection<rT> > &camProjs,
                             octomap::OccupancyOcTreeBase<nT> *ocTree );

protected:
    template < typename pT >
    std::shared_ptr<CReal> copy_pcl_point_cloud_2_real(
            const pcl::PointCloud<pT> &pc ) const;

    void copy_cam_proj_params(const CameraProjection<rT> &camProj, CReal *cp) const;
    void collect_depth_mask_and_pcl_point_cloud(const CMask *visMask, const CReal *pixels,
            int n, CReal *bfDepthMap, int *bfMaskIdxMap, int &bfSize,
            pcl::PointCloud<pcl::PointXY> &pc ) const;
    int update_visibility_mask_by_2D( const pcl::PointCloud<pcl::PointXY>::Ptr pPC,
                                      const CReal *depthMap,
                                      const int *maskIdxMap,
                                      CMask *visMask ) const;
    template < typename pT >
    void convert_pcl_2_octomap_by_vis_mask( const pcl::PointCloud<pT> &pclPC,
            const CMask *visMask, octomap::Pointcloud &opc, int nApproximateVisible=0 ) const;

protected:
    rT occThreshold;
    int knnK;
    int knnOcc;
};

template < typename rT >
void OccupancyMapBuilder<rT>::set_occlusion_threshold(rT t) {
    assert(t > 0);
    occThreshold = t;
}

template < typename rT >
void OccupancyMapBuilder<rT>::set_knn_k_and_knn_occ(int k, int occ) {
    assert( k > 0 );
    assert( occ > 0 );
    assert( k >= occ );

    knnK = k;
    knnOcc = occ;
}

template < typename rT >
template < typename pT >
std::shared_ptr<CReal> OccupancyMapBuilder<rT>::copy_pcl_point_cloud_2_real(
        const pcl::PointCloud<pT> &pc ) const {
    const int N = pc.size();
    assert(N > 0);

    std::shared_ptr<CReal> pPC ( new CReal[ N*3 ], [](CReal *p){ delete [] p; } );

    CReal *ptr = pPC.get();
    int pcIdx = 0;
    for ( int i = 0; i < N; ++i ) {
        ptr[pcIdx]   = pc[i].x;
        ptr[pcIdx+1] = pc[i].y;
        ptr[pcIdx+2] = pc[i].z;

        pcIdx += 3;
    }

    return pPC;
}

template < typename rT >
void OccupancyMapBuilder<rT>::copy_cam_proj_params(
        const CameraProjection<rT> &camProj, CReal *cp) const {
    Eigen::Matrix3<rT> RRC = camProj.R * camProj.RC;

    cp[0] = camProj.K(0, 0); cp[1] = camProj.K(0, 1); cp[2] = camProj.K(0, 2);
    cp[3] = camProj.K(1, 0); cp[4] = camProj.K(1, 1); cp[5] = camProj.K(1, 2);
    cp[6] = camProj.K(2, 0); cp[7] = camProj.K(2, 1); cp[8] = camProj.K(2, 2);

    cp[ 9] = RRC(0, 0); cp[10] = RRC(0, 1); cp[11] = RRC(0, 2);
    cp[12] = RRC(1, 0); cp[13] = RRC(1, 1); cp[14] = RRC(1, 2);
    cp[15] = RRC(2, 0); cp[16] = RRC(2, 1); cp[17] = RRC(2, 2);

    cp[18] = camProj.T(0); cp[19] = camProj.T(1); cp[20] = camProj.T(2);
}

template < typename rT >
void OccupancyMapBuilder<rT>::collect_depth_mask_and_pcl_point_cloud(
        const CMask *visMask, const CReal *pixels,
        int n, CReal *bfDepthMap, int *bfMaskIdxMap, int &bfSize,
        pcl::PointCloud<pcl::PointXY> &pc ) const {
    bfSize = 0;

    for ( int i = 0; i < n; ++i ) {
        if ( visMask[i] == 0 ) {
            continue;
        }

        bfDepthMap[bfSize] = pixels[ i*3 + 2 ];
        bfMaskIdxMap[bfSize] = i;
        bfSize++;

        pc.emplace_back( pixels[ i*3 ], pixels[ i*3 + 1 ] );
    }
}

template < typename rT >
int OccupancyMapBuilder<rT>::update_visibility_mask_by_2D( const pcl::PointCloud<pcl::PointXY>::Ptr pPC,
                                  const CReal *depthMap,
                                  const int *maskIdxMap,
                                  CMask *visMask ) const {
    // Create a KD-Tree.
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr tree ( new pcl::KdTreeFLANN<pcl::PointXY> );
    tree->setInputCloud(pPC);

    // Buffers for the KNN.
    std::vector<int> indexKNN(knnK+1);
    std::vector<float> squaredDistance(knnK+1);

    const int N = pPC->size();
    int n; // Number of neighbors found.
    int visibleCount = 0;
    for ( int i = 0; i < N; ++i ) {
        indexKNN.clear();
        squaredDistance.clear();

        n = tree->nearestKSearch( *pPC, i, knnK+1, indexKNN, squaredDistance );

        if (0 == n) {
            // No neighbors, visible. Should not be here.
            visMask[ maskIdxMap[i] ] = OCP_MAP_CAM_VISIBLE;
            continue;
        }

        int closerCount = 0;
        int idx; // Index into depthMap of a neighbor.
        const rT d = depthMap[i];

        // Loop over neighbors.
        for ( int j = 0; j < n; ++j ) {
            idx = indexKNN[j];

            if ( d - depthMap[idx] >= occThreshold ) {
                // Occlusion.
                closerCount++;
            }
        }

        if ( closerCount >= knnOcc ) {
            visMask[ maskIdxMap[i] ] = OCP_MAP_CAM_INVISIBLE;
        } else {
            visMask[ maskIdxMap[i] ] = OCP_MAP_CAM_VISIBLE;
            visibleCount++;
        }
    }

    return visibleCount;
}

template < typename rT >
template < typename pT >
void OccupancyMapBuilder<rT>::convert_pcl_2_octomap_by_vis_mask(
        const pcl::PointCloud<pT> &pclPC, const CMask *visMask,
        octomap::Pointcloud &opc, int nApproximateVisible ) const {
    const int N = pclPC.size(); assert( N > 0 );
    // Reserve memory for the octomap::Pointcloud object.
    if ( nApproximateVisible <= 0 ) {
        opc.reserve(N);
    } else {
        opc.reserve(nApproximateVisible);
    }

    // Fill opc by referring to visMask.
    for ( int i = 0; i < N; ++i) {
        if ( visMask[i] == OCP_MAP_CAM_VISIBLE ) {
            const pT &point = pclPC[i];
            opc.push_back( octomap::point3d(
                    point.x, point.y, point.z ) );
        }
    }
}

template < typename rT >
template < typename pT, typename nT >
void OccupancyMapBuilder<rT>::build_occupancy_map(
        const typename pcl::PointCloud<pT>::Ptr pInCloud,
        const std::vector<CameraProjection<rT> > &camProjs,
        octomap::OccupancyOcTreeBase<nT> *ocTree) {
    QUICK_TIME_START(teBuild)
    const int N = pInCloud->size();

    // Convert everything into CReal type.
    std::shared_ptr<CReal> pPC = copy_pcl_point_cloud_2_real(*pInCloud);

    // Allocation for the buffers.
    auto bfDepthMap   = new CReal[ N ];
    auto bfMaskIdxMap = new int[ N ];
    int bfSize = 0;

    // OccupancyMapBuilder.
    CR_VisMask crVisMask(N);
    crVisMask.copy_point_cloud(pPC.get(), N);

    const int nC = camProjs.size();
    for ( int i = 0; i < nC; ++i ) {
        std::cout << i << ": Cam ID " << camProjs[i].id << ". " ;
        copy_cam_proj_params( camProjs[i], crVisMask.get_u_cam_proj() );
        crVisMask.set_cam_proj_size( camProjs[i].height, camProjs[i].width );
        QUICK_TIME_START(teCrVisMask)
        crVisMask.cr_update_visibility_mask();
        QUICK_TIME_END(teCrVisMask)
        std::cout << "Proj " << teCrVisMask << " ms. ";

        pcl::PointCloud<pcl::PointXY>::Ptr pPC2D (new pcl::PointCloud<pcl::PointXY>);
        collect_depth_mask_and_pcl_point_cloud(
                crVisMask.get_vis_mask(), crVisMask.get_pixels(), N,
                bfDepthMap, bfMaskIdxMap, bfSize, *pPC2D );

        if ( bfSize == 0 ) {
            std::cout << "No points visible. \n";
            continue;
        }

        // Update visibility map.
        QUICK_TIME_START(teUpdateVisMask)
        std::cout << "Local occ chk " << bfSize << ". ";
        int nVisible = update_visibility_mask_by_2D( pPC2D, bfDepthMap, bfMaskIdxMap, crVisMask.get_vis_mask() );
        QUICK_TIME_END(teUpdateVisMask)
        std::cout << nVisible << " visible. "<< teUpdateVisMask << " ms. ";

        // Create octomap::Pointcloud.
        octomap::Pointcloud ocPC;
        convert_pcl_2_octomap_by_vis_mask( *pInCloud, crVisMask.get_vis_mask(), ocPC, nVisible );

        // Origin point for an octomap::Pointcloud object.
        const Eigen::Vector3f &camT = camProjs[i].T;
        octomap::point3d origin( camT(0), camT(1), camT(2) );

        // Insert the point cloud into the current octomap.
        QUICK_TIME_START(teInsertPC)
        ocTree->insertPointCloud( ocPC, origin );
        QUICK_TIME_END(teInsertPC)
        std::cout << "Insert to octomap " << teInsertPC << " ms. \n";
    }

    // Clean up.
    delete [] bfMaskIdxMap;
    delete [] bfDepthMap;

    QUICK_TIME_END(teBuild)
    std::cout << "Build the occupancy map in " << teBuild << " ms. \n";
}

} // namespace pcu

#endif //POINTCLOUDUTILS_INCLUDES_HOLEBOUNDARYDETECTION_OCCUPANCYMAPBUILDER_HPP
