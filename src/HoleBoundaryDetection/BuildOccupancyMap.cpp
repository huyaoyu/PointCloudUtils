//
// Created by yaoyu on 5/24/20.
//

#include <cstdint>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>

#include "Args/Args.hpp"
#include "CameraGeometry/CameraProjection.hpp"
#include "DataInterfaces/Plain/FromVector.hpp"
#include "Exception/Common.hpp"
#include "OccupancyMap/OccupancyMap.hpp"
#include "PCCommon/IO.hpp"
#include "Profiling/SimpleTime.hpp"

// Namespaces.
namespace bpo = boost::program_options;

// Local types for convenience.
typedef float Real_t;
typedef CameraProjection<Real_t> CamProj_t;
typedef std::vector<CamProj_t> CPs_t;
typedef pcl::PointXYZ P_t;
typedef pcl::PointCloud<P_t>::Ptr PCPtr_t;

typedef std::uint8_t VT_t; // Visibility mask type.
static const VT_t VISIBLE   = pcu::OCP_MAP_CAM_VISIBLE;
static const VT_t INVISIBLE = pcu::OCP_MAP_CAM_INVISIBLE;

/************************ Command line arguments. ****************************/
class Args
{
public:
    Args()
        : ocpMapResolution(0.05),
          flagCUDA(false)
          {}

    ~Args() = default;

    bool validate() const {
        bool flag = true;

        flag = ocpMapResolution > 0 ? true : false;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << args.AS_IN_MVS       << ": " << args.inMVS            << "\n";
        out << args.AS_IN_LIDAR     << ": " << args.inLiDAR          << "\n";
        out << args.AS_IN_CAM_PROJ  << ": " << args.inCamProj        << "\n";
        out << args.AS_OUT_DIR      << ": " << args.outDir           << "\n";
        out << args.AS_OUT_OT_NAME  << ": " << args.outOTName        << "\n";
        out << args.AS_OUT_CSV_NAME << ": " << args.outCSVName       << "\n";
        out << args.AS_OCP_MAP_RES  << ": " << args.ocpMapResolution << "\n";
        out << args.AS_FLAG_CUDA    << ": " << args.flagCUDA         << "\n";

        return out;
    }

    void parse_args(int argc, char* argv[]) {
        try
        {
            bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

            optDesc.add_options()
                    ("help", "Produce help message.")
                    (AS_IN_MVS.c_str(), bpo::value< std::string >(&inMVS)->required(), "The input MVS point cloud. ")
                    (AS_IN_LIDAR.c_str(), bpo::value< std::string >(&inLiDAR)->required(), "The input LiDAR point cloud. ")
                    (AS_IN_CAM_PROJ.c_str(), bpo::value< std::string >(&inCamProj)->required(), "The input JSON file that records the camera projection objects. ")
                    (AS_OUT_DIR.c_str(), bpo::value< std::string >(&outDir)->required(), "The output file. ")
                    (AS_OUT_OT_NAME.c_str(), bpo::value< std::string >(&outOTName)->default_value("OcMap.ot"), "The output filename of the octomap object. ")
                    (AS_OUT_CSV_NAME.c_str(), bpo::value< std::string >(&outCSVName)->default_value("Voxels.csv"), "The output filename of the CSV file for the voxels. ")
                    (AS_OCP_MAP_RES.c_str(), bpo::value< double >(&ocpMapResolution)->default_value(0.05), "The resolution of the occupancy map. ")
                    (AS_FLAG_CUDA.c_str(), bpo::value< int >()->implicit_value(1), "Set this flag for CUDA support. ");

            bpo::positional_options_description posOptDesc;
            posOptDesc.add(Args::AS_IN_MVS.c_str(), 1
            ).add(Args::AS_IN_LIDAR.c_str(), 1
            ).add(Args::AS_IN_CAM_PROJ.c_str(), 1
            ).add(Args::AS_OUT_DIR.c_str(), 1);

            bpo::variables_map optVM;
            bpo::store(bpo::command_line_parser(argc, argv).
                    options(optDesc).positional(posOptDesc).run(), optVM);
            bpo::notify(optVM);

            if ( optVM.count(Args::AS_FLAG_CUDA.c_str()) ) {
                flagCUDA = true;
            } else {
                flagCUDA = false;
            }
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
            throw(e);
        }

        if ( !validate() ) {
            EXCEPTION_INVALID_ARGUMENTS_IN_CLASS()
        }
    }

public:
    const std::string AS_IN_MVS       = "in-mvs";
    const std::string AS_IN_LIDAR     = "in-lidar";
    const std::string AS_IN_CAM_PROJ  = "in-cam-proj";
    const std::string AS_OUT_DIR      = "out-dir";
    const std::string AS_OUT_OT_NAME  = "out-ot-name";
    const std::string AS_OUT_CSV_NAME = "out-csv-name";
    const std::string AS_OCP_MAP_RES  = "ocp-map-resolution";
    const std::string AS_FLAG_CUDA    = "cuda";

    std::string inMVS;
    std::string inLiDAR;
    std::string inCamProj;
    std::string outDir; // The output directory.
    std::string outOTName; // The output octomap filename.
    std::string outCSVName; // The output CSV filename.
    double ocpMapResolution;
    bool flagCUDA;
};

template < typename rT >
static int update_visibility_mask_by_2D( const pcl::PointCloud<pcl::PointXY>::Ptr pPC,
                                         const rT *depthMap,
                                         const int *maskIdxMap,
                                         Eigen::MatrixX<VT_t> &visMask ,
                                         rT occThreshold=0.05 ) {
    const int k    = 18;  // KNN.
    const int kOcc = k/3; // Occlusion limit.

    // Create a KD-Tree.
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr tree ( new pcl::KdTreeFLANN<pcl::PointXY> );
    tree->setInputCloud(pPC);

    // Buffers for the KNN.
    std::vector<int> indexKNN(k+1);
    std::vector<float> squaredDistance(k+1);

    const int N = pPC->size();
    int n; // Number of neighbors found.
    int visibleCount = 0;
    for ( int i = 0; i < N; ++i ) {
        indexKNN.clear();
        squaredDistance.clear();

        n = tree->nearestKSearch( *pPC, i, k+1, indexKNN, squaredDistance );

        if (0 == n) {
            // No neighbors, visible. Should not be here.
            visMask( maskIdxMap[i], 0 ) = VISIBLE;
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

        if ( closerCount >= kOcc ) {
            visMask( maskIdxMap[i], 0 ) = INVISIBLE;
        } else {
            visMask( maskIdxMap[i], 0 ) = VISIBLE;
            visibleCount++;
        }
    }

    return visibleCount;
}

template < typename pT, typename rT >
static int update_visibility_mask( const typename pcl::PointCloud<pT>::Ptr pInCloud,
                                   const CameraProjection<rT> &camProj,
                                   Eigen::MatrixX<VT_t> &visMask,
                                   rT *bfDepthMap, int *bfMaskIdxMap) {
    assert( bfDepthMap != nullptr );
    assert( bfMaskIdxMap != nullptr );

    pcl::PointCloud<pcl::PointXY>::Ptr pPC2D (new pcl::PointCloud<pcl::PointXY>);

    const int N = pInCloud->size();

    // Fill depthMap, maskIdxMap, and pPC2D.
    QUICK_TIME_START(teBuild2D)
    Eigen::Vector3<rT> wp, sp, pixel;
    int nInFOV = 0; // Number of points that fall into the FOV of the camera.
    for ( int i = 0; i < N; ++i ) {
        pT &point = pInCloud->at(i);
        wp<< point.x, point.y, point.z;
        sp = camProj.RCtRt * ( wp - camProj.T );

        bool flag = camProj.is_world_point_in_image(wp);

        if ( sp(2) <= 0 ) {
            continue;
        }

        pixel(0) = camProj.K(0, 0) * sp(0)  / sp(2) + camProj.K(0,2);
        pixel(1) = camProj.K(1, 1) * sp(1)  / sp(2) + camProj.K(1,2);

        if ( pixel(0) < 0 || pixel(0) > camProj.width ) {
            continue;
        }
        if ( pixel(1) < 0 || pixel(1) > camProj.height ) {
            continue;
        }

        bfDepthMap[nInFOV] = sp(2);
        bfMaskIdxMap[nInFOV] = i;

        pPC2D->emplace_back( pixel(0), pixel(1) );

        nInFOV++;
    }

    QUICK_TIME_END(teBuild2D)
    std::cout << "Build 2D point cloud in " << teBuild2D << " ms. " << "\n";

    if ( 0 == nInFOV ) {
        return 0;
    }

    // Update visibility map.
    QUICK_TIME_START(teUpdateVisMask)
    std::cout << "Check local occlusion for " << nInFOV << " points. \n";
    int nVisible = update_visibility_mask_by_2D( pPC2D, bfDepthMap, bfMaskIdxMap, visMask );
    QUICK_TIME_END(teUpdateVisMask)
    std::cout << nVisible << " points visible. \n";
    std::cout << "Update visibility mask by 2D point cloud in "
              << teUpdateVisMask << " ms. \n";

    return nVisible;
}

template < typename pT, typename rT >
static void build_occupancy_map(const typename pcl::PointCloud<pT>::Ptr pInCloud,
                                const std::vector< CameraProjection<rT> > &camProjs,
                                pcu::OccupancyMap &ocpMap ) {
    QUICK_TIME_START(te)

    const int nP = pInCloud->size();
    const int nC = camProjs.size();
    assert( nP > 0 );
    assert( nC > 0 );

    Eigen::MatrixX<VT_t> visMask(nP, 1);
    const Eigen::MatrixX<VT_t> visMaskClean = Eigen::MatrixX<VT_t>::Constant(nP, 1, INVISIBLE);

    // Buffers.
    rT  *bfDepthMap   = new rT[nP];
    int *bfMaskIdxMap = new int[nP];

    for ( int i = 0; i < 2; ++i ) {
        std::cout << "Camera " << i << "\n";
        visMask = visMaskClean;
        update_visibility_mask<pT, rT>(
                pInCloud, camProjs[i], visMask, bfDepthMap, bfMaskIdxMap );
    }

    delete [] bfMaskIdxMap;
    delete [] bfDepthMap;

    QUICK_TIME_END(te)
    std::cout << "Build occupancy map in " << te << " ms. \n";
}

static CPs_t read_cam_projs( const std::string &fn ) {
    CPs_t camProjs = read_cam_proj_from_json<Real_t>(fn);

    // Test use.
    std::cout << camProjs.size() << " camera projection objects read from JSON file. \n";
    std::cout << "camProjs[last] = \n"
              << camProjs[ camProjs.size() - 1 ] << "\n";

    return camProjs;
}

static PCPtr_t read_and_merge_pcl_point_clouds(
        const std::string &fnMVS,
        const std::string &fnLiDAR,
        PCPtr_t &pLiDAR ) {
    // Read the MVS and LiDAR point clouds.
    PCPtr_t pMVS = pcu::read_point_cloud<P_t>(fnMVS);
//    PCPtr_t pLiDAR = pcu::read_point_cloud<P_t>(fnLiDAR);
    pLiDAR = pcu::read_point_cloud<P_t>(fnLiDAR);

    // Merge the MVS and LiDAR point clouds.
    PCPtr_t pMerged ( new pcl::PointCloud<P_t> );
    *pMerged += *pMVS;
    *pMerged += *pLiDAR;

    // Test use.
    std::cout << "pMVS->size()    = " << pMVS->size()    << "\n";
    std::cout << "pLiDAR->size()  = " << pLiDAR->size()  << "\n";
    std::cout << "pMerged->size() = " << pMerged->size() << "\n";

    return pMerged;
}

static void build( const PCPtr_t pMerged,
        const PCPtr_t pLiDAR,
        const CPs_t &camProjs,
        pcu::OccupancyMap &ocpMap,
        bool flagCUDA=false ) {
    // Build the occupancy map.
    std::cout << "Start building the occupancy map. \n";
    if ( flagCUDA ) {
        ocpMap.build_from_pcl_and_cam_proj<P_t, Real_t>(pMerged, camProjs);
    } else {
        build_occupancy_map<P_t, Real_t>( pMerged, camProjs, ocpMap );
    }

    ocpMap.update_occupancy_by_pcl<P_t>(pLiDAR);

    std::cout << "octomap has " << ocpMap.get_octree().getNumLeafNodes() << " leaf nodes. \n";

    // Find frontiers.
    std::cout << "Start finding frontiers. \n";
    ocpMap.find_frontiers();
}

int main(int argc, char** argv) {
    QUICK_TIME_START(teMain)
    std::cout << "Hello, BuildOccupancyMap! \n";
    MAIN_COMMON_LINES_ONE_CLASS(argc, argv, args)

    // Read the camera projection objects.
    CPs_t camProjs = read_cam_projs(args.inCamProj);

    // Read the MVS and LiDAR point clouds and merge.
    PCPtr_t pLiDAR;
    PCPtr_t pMerged = read_and_merge_pcl_point_clouds( args.inMVS, args.inLiDAR, pLiDAR );

    // Occupancy map.
    pcu::OccupancyMap ocMap;
    ocMap.initialize(0.05);
    build( pMerged, pLiDAR, camProjs, ocMap, args.flagCUDA );

    // Write the occupancy map and frontier CSV..
    {
        std::string outFn = args.outDir + "/" + args.outOTName;
        ocMap.write(outFn);
        outFn = args.outDir + "/" + args.outCSVName;
        ocMap.write_voxels_as_list(outFn);
    }

    QUICK_TIME_SHOW(teMain, "BuildOccupancyMap")
    return 0;
}

