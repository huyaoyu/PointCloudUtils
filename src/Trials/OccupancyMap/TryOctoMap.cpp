//
// Created by yaoyu on 5/7/20.
//

#define ENABLE_PROFILE 0

#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include <boost/program_options.hpp>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "Args/Args.hpp"
#include "CameraGeometry/CameraProjection.hpp"
#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "DataInterfaces/JSONHelper/Reader.hpp"
#include "DataInterfaces/Plain/FromVector.hpp"
#include "Exception/Common.hpp"
#include "OccupancyMap/OccupancyMap.hpp"
#include "PCCommon/IO.hpp"
#include "PCCommon/extraction.hpp"
#include "Profiling/Instrumentor.hpp"
#include "Profiling/SimpleTime.hpp"

// Namespaces.
namespace bpo = boost::program_options;
using JSON = nlohmann::json;

struct NoCamProjFound : virtual exception_common_base {};

typedef std::uint8_t VT;

static const VT VISIBLE   = 1;
static const VT INVISIBLE = 0;

class Args
{
public:
    Args(): flagCUDA(false) {}

    ~Args() = default;

    bool validate() {
        bool flag = true;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_MVS << ": " << args.inMVS << std::endl;
        out << Args::AS_IN_LIDAR << ": " << args.inLiDAR << std::endl;
        out << Args::AS_IN_CAM_PROJ << ": " << args.inCamProj << std::endl;
        out << Args::AS_OUT_DIR << ": " << args.outDir << std::endl;
        out << Args::AS_FLAG_CUDA << ": " << args.flagCUDA << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_MVS;
    static const std::string AS_IN_LIDAR;
    static const std::string AS_IN_CAM_PROJ;
    static const std::string AS_OUT_DIR;
    static const std::string AS_FLAG_CUDA;

public:
    std::string inMVS;
    std::string inLiDAR;
    std::string inCamProj;
    std::string outDir; // The output directory.
    bool flagCUDA;
};

const std::string Args::AS_IN_MVS      = "inMVS";
const std::string Args::AS_IN_LIDAR    = "inLiDAR";
const std::string Args::AS_IN_CAM_PROJ = "inCamProj";
const std::string Args::AS_OUT_DIR     = "outDir";
const std::string Args::AS_FLAG_CUDA   = "cuda";

static void parse_args(int argc, char* argv[], Args& args) {

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_MVS.c_str(), bpo::value< std::string >(&args.inMVS)->required(), "The input MVS point cloud.")
                (Args::AS_IN_LIDAR.c_str(), bpo::value< std::string >(&args.inLiDAR)->required(), "The input LiDAR point cloud.")
                (Args::AS_IN_CAM_PROJ.c_str(), bpo::value< std::string >(&args.inCamProj)->required(), "The input JSON file that records the camera projection objects.")
                (Args::AS_OUT_DIR.c_str(), bpo::value< std::string >(&args.outDir)->required(), "The output file.")
                (Args::AS_FLAG_CUDA.c_str(), bpo::value< int >()->implicit_value(1), "Set this flag for CUDA support.");

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
            args.flagCUDA = true;
        } else {
            args.flagCUDA = false;
        }
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        throw(e);
    }

    if ( !args.validate() ) {
        EXCEPTION_INVALID_ARGUMENTS(args)
    }
}

template < typename rT >
static void read_cam_proj_from_json( const std::string &fn,
        std::vector<CameraProjection<rT>> &camProjs ) {

    std::shared_ptr<JSON> pJson = read_json( fn );

    const auto& jCamProjs = (*pJson)["camProjs"];
    const int N = jCamProjs.size();

    if ( 0 == N ) {
        BOOST_THROW_EXCEPTION( NoCamProjFound() << ExceptionInfoString("No camera projection objects found in the JSON file.") );
    }

    for ( int i = 0; i < N; ++i ) {
        CameraProjection<rT> camProj;
        const auto& jCamProj = jCamProjs[i];

        camProj.id     = jCamProj["id"];
        camProj.height = jCamProj["height"];
        camProj.width  = jCamProj["width"];

        convert_vector_2_eigen_mat3(   jCamProj["K"].get<  std::vector<rT> >(), camProj.K );
        convert_vector_2_eigen_mat3(   jCamProj["RC"].get< std::vector<rT> >(), camProj.RC );
        convert_vector_2_eigen_mat3(   jCamProj["R"].get<  std::vector<rT> >(), camProj.R );
        convert_vector_2_eigen_vector( jCamProj["T"].get<  std::vector<rT> >(), camProj.T );

        std::vector<rT> qv = jCamProj["Q"].get< std::vector<rT> >();

        camProj.Q = Eigen::Quaternion<rT>( qv[0], qv[1], qv[2], qv[3] );

        camProj.update_RCtRt();
        camProj.update_frustum_normals();

        camProjs.emplace_back( camProj );
    }
}

template < typename rT >
static int update_visibility_mask_by_2D( const pcl::PointCloud<pcl::PointXY>::Ptr pPC,
        const rT *depthMap,
        const int *maskIdxMap,
        Eigen::MatrixX<VT> &visMask ,
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
        Eigen::MatrixX<VT> &visMask,
        rT *bfDepthMap, int *bfMaskIdxMap) {
    assert( bfDepthMap != nullptr );
    assert( bfMaskIdxMap != nullptr );

    pcl::PointCloud<pcl::PointXY>::Ptr pPC2D (new pcl::PointCloud<pcl::PointXY>);

    const int N = pInCloud->size();

    // Fill depthMap, maskIdxMap, and pPC2D.
    QUICK_TIME_START(teBuild2D)
    Eigen::Vector3<rT> wp, sp, pixel;
    int nInFOV = 0; // Number of points that fall into the FOV of the camera.
//    PROFILE_SCOPE("ForAllPoints")
    for ( int i = 0; i < N; ++i ) {
        pT &point = pInCloud->at(i);
        wp<< point.x, point.y, point.z;
        sp = camProj.RCtRt * ( wp - camProj.T );

//        if ( 422416 == i ) {
//            std::cout << "Ha" << std::endl;
//        }

        bool flag = camProj.is_world_point_in_image(wp);

        if ( sp(2) <= 0 ) {
            if ( flag ) {
                std::cout << "O!" << std::endl;
            }
            continue;
        }

        pixel(0) = camProj.K(0, 0) * sp(0)  / sp(2) + camProj.K(0,2);
        pixel(1) = camProj.K(1, 1) * sp(1)  / sp(2) + camProj.K(1,2);
//        pixel(2) = 1.0;

        if ( pixel(0) < 0 || pixel(0) > camProj.width ) {
            if ( flag ) {
                std::cout << "O!" << std::endl;
            }
            continue;
        }
        if ( pixel(1) < 0 || pixel(1) > camProj.height ) {
            if ( flag ) {
                std::cout << "O!" << std::endl;
            }
            continue;
        }

        bfDepthMap[nInFOV] = sp(2);
        bfMaskIdxMap[nInFOV] = i;

        {
    //                PROFILE_SCOPE("pPC2D.emplace_back")
            pPC2D->emplace_back( pixel(0), pixel(1) );
        }

        nInFOV++;
    }

    QUICK_TIME_END(teBuild2D)
    std::cout << "Build 2D point cloud in " << teBuild2D << " ms. " << std::endl;

    if ( 0 == nInFOV ) {
        return 0;
    }

    // Update visibility map.
    QUICK_TIME_START(teUpdateVisMask)
    std::cout << "Check local occlusion for " << nInFOV << " points. " << std::endl;
    int nVisible = update_visibility_mask_by_2D( pPC2D, bfDepthMap, bfMaskIdxMap, visMask );
    QUICK_TIME_END(teUpdateVisMask)
    std::cout << nVisible << " points visible. " << std::endl;
    std::cout << "Update visibility mask by 2D point cloud in "
              << teUpdateVisMask << " ms. " << std::endl;

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

    Eigen::MatrixX<VT> visMask(nP, 1);
    const Eigen::MatrixX<VT> visMaskClean = Eigen::MatrixX<VT>::Constant(nP, 1, INVISIBLE);

#if ENABLE_PROFILE
    Instrumentor::Get().BeginSession("Profile_BuildOcpMap2D", "Profile_BuildOcpMap2D.json");
#endif

    // Buffers.
    rT  *bfDepthMap   = new rT[nP];
    int *bfMaskIdxMap = new int[nP];
#if ENABLE_PROFILE
    for ( int i = 0; i < 4; ++i ) {
        PROFILE_SCOPE("update_visibility_mask")
#else
    for ( int i = 0; i < 2; ++i ) {
#endif
        std::cout << "Camera " << i << std::endl;
        visMask = visMaskClean;
        update_visibility_mask<pT, rT>(
                pInCloud, camProjs[i], visMask, bfDepthMap, bfMaskIdxMap );
    }

    delete [] bfMaskIdxMap;
    delete [] bfDepthMap;

#if ENABLE_PROFILE
    Instrumentor::Get().EndSession();
    // Test use.
    std::stringstream ss;
    ss << "Profiling. Throw exception on line " << __LINE__;
    throw std::runtime_error(ss.str());
#endif

    QUICK_TIME_END(te)
    std::cout << "Build occupancy map in " << te << " ms. " << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "Hello, TryOctoMap! " << std::endl;

    // Handle the command line.
    MAIN_COMMON_LINES(argc, argv, args)

    // Read the camera projection objects.
    std::vector<CameraProjection<float>> camProjs;
    read_cam_proj_from_json(args.inCamProj, camProjs);

    // Test use.
    std::cout << camProjs.size() << " camera projection objects read from JSON file. " << std::endl;
    std::cout << "camProjs[last] = " << std::endl
              << camProjs[ camProjs.size() - 1 ] << std::endl;

    // Read the MVS and LiDAR point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pMVS =
            pcu::read_point_cloud<pcl::PointXYZ>(args.inMVS);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pLiDAR =
            pcu::read_point_cloud<pcl::PointXYZ>(args.inLiDAR);

    // Merge the MVS and LiDAR point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pMerged ( new pcl::PointCloud<pcl::PointXYZ> );
    *pMerged += *pMVS;
    *pMerged += *pLiDAR;

    // Test use.
    std::cout << "pMVS->size()    = " << pMVS->size() << std::endl;
    std::cout << "pLiDAR->size()  = " << pLiDAR->size() << std::endl;
    std::cout << "pMerged->size() = " << pMerged->size() << std::endl;

    // Occupancy map.
    pcu::OccupancyMap ocMap;
    ocMap.initialize(0.05);

    // Build the occupancy map.
    std::cout << "Start building the occupancy map." << std::endl;
    if ( args.flagCUDA ) {
        ocMap.build_from_pcl_and_cam_proj<pcl::PointXYZ, float>(pMerged, camProjs);
    } else {
        build_occupancy_map<pcl::PointXYZ, float>( pMerged, camProjs, ocMap );
    }

    // Write the occupancy map.
    {
        std::string outFn = args.outDir + "/OcMap.ot";
        ocMap.write(outFn);
    }

    return 0;
}