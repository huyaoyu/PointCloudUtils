//
// Created by yaoyu on 5/26/20.
//

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Args/Args.hpp"
#include "CameraGeometry/CameraProjection.hpp"
#include "CameraGeometry/CameraPose2PCL.hpp"
#include "CameraGeometry/IO.hpp"
#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "DataInterfaces/Plain/Matrix.hpp"
#include "Exception/Common.hpp"
#include "Filesystem/Filesystem.hpp"
#include "PCCommon/BBox.hpp"
#include "PCCommon/IO.hpp"
#include "Profiling/SimpleTime.hpp"
#include "Visualization/Print.hpp"

// Namespaces.
namespace bpo = boost::program_options;

// Local exceptions.
struct NoCameraFound : virtual exception_common_base {};

// Local typedefs.
typedef pcl::PointXYZ P_t;
typedef pcl::PointCloud<P_t> PC_t;
typedef PC_t::Ptr PCPtr_t;
typedef CameraProjection<float> CP_t;
typedef std::vector< CP_t > CPs_t;

/************************ Command line arguments. ****************************/
class Args
{
public:
    Args() = default;
    ~Args() = default;

    bool validate() const {
        bool flag = true;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << args.AS_IN_CLOUD     << ": " << args.inCloud    << "\n";
        out << args.AS_IN_CAM_POSES << ": " << args.inCamPoses << "\n";
        out << args.AS_IN_CAM_P1    << ": " << args.inCamP1    << "\n";
        out << args.AS_IN_IMG_SIZE  << ": " << args.inImgSize  << "\n";
        out << args.AS_OUT_DIR      << ": " << args.outDir     << "\n";

        return out;
    }

    void parse_args(int argc, char* argv[]) {
        try
        {
            bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

            optDesc.add_options()
                    ("help", "Produce help message.")
                    (AS_IN_CLOUD.c_str(), bpo::value< std::string >(&inCloud)->required(), "The input point cloud. ")
                    (AS_IN_CAM_POSES.c_str(), bpo::value<std::string>(&inCamPoses)->required(), "The input camera pose CSV file. ")
                    (AS_IN_CAM_P1.c_str(), bpo::value<std::string>(&inCamP1)->required(), "The input camera P1 matrix. ")
                    (AS_IN_IMG_SIZE.c_str(), bpo::value<std::string>(&inImgSize)->required(), "The input image size JSON file. ")
                    (AS_OUT_DIR.c_str(), bpo::value< std::string >(&outDir)->required(), "The output file. ");

            bpo::positional_options_description posOptDesc;
            posOptDesc.add(Args::AS_IN_CLOUD.c_str(), 1
            ).add(Args::AS_IN_CAM_POSES.c_str(), 1
            ).add(Args::AS_IN_CAM_P1.c_str(), 1
            ).add(Args::AS_IN_IMG_SIZE.c_str(), 1
            ).add(Args::AS_OUT_DIR.c_str(), 1);

            bpo::variables_map optVM;
            bpo::store(bpo::command_line_parser(argc, argv).
                    options(optDesc).positional(posOptDesc).run(), optVM);
            bpo::notify(optVM);
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
    const std::string AS_IN_CLOUD     = "in-cloud";
    const std::string AS_IN_CAM_POSES = "in-cam-poses";
    const std::string AS_IN_CAM_P1    = "in-cam-p1";
    const std::string AS_IN_IMG_SIZE  = "in-img-size";
    const std::string AS_OUT_DIR      = "out-dir";

    std::string inCloud;
    std::string inCamPoses; // The input camera poses.
    std::string inCamP1;    // The camera intrinsics, 3x4 matrix. The 3x3 part is K.
    std::string inImgSize;  // The JSON file contains the image size.
    std::string outDir;     // The output directory.
};

template < typename rT >
static void read_image_size_json( const std::string &fn,
                                  rT &height, rT &width ) {
    using json = nlohmann::json;

    std::ifstream ifs(fn);
    json jExt;
    ifs >> jExt;

    height = jExt["height"];
    width  = jExt["width"];
}

static Eigen::Matrix3f read_camera_intrinsics_from_p1(
        const std::string &fn ) {
    Eigen::MatrixXf camP1;
    read_matrix( fn, 3, 4, " ", camP1 );
    Eigen::Matrix3f camK = camP1.block(0,0,3,3);

    return camK;
}

template < typename rT >
static void set_image_size_for_camera_projection_objects(
        std::vector< CameraProjection<rT> > &camProjs,
        rT height, rT width ) {
    for ( auto& cp : camProjs ) {
        cp.height = height;
        cp.width  = width;
        cp.update_frustum_normals();
    }
}

template < typename pT, typename rT >
static void find_corner_points_from_bbox( const pT &p0,
                                          const pT &p1,
                                          Eigen::MatrixX<rT> &points ) {
    points = Eigen::MatrixX<rT>::Zero(3, 8);
    points << p0.x, p1.x, p1.x, p0.x, p0.x, p1.x, p1.x, p0.x,
            p0.y, p0.y, p1.y, p1.y, p0.y, p0.y, p1.y, p1.y,
            p0.z, p0.z, p0.z, p0.z, p1.z, p1.z, p1.z, p1.z;
}

static CPs_t read_camera_projs( const std::string &imgSizeFn,
        const CamPoseCSVRepresentation<float> &cpCSVR,
        const Eigen::Matrix3f &camK ) {
    float imgHeight, imgWidth;
    read_image_size_json( imgSizeFn, imgHeight, imgWidth );
    std::cout << "Image size is ( " << imgHeight << ", " << imgWidth << " ). \n";

    CPs_t camProjOri;
    convert_from_quaternion_translation_table( cpCSVR.id, cpCSVR.quat, cpCSVR.pos, camK, camProjOri );
    set_image_size_for_camera_projection_objects( camProjOri, imgHeight, imgWidth );

    return camProjOri;
}

template < typename pT, typename rT >
static void filter_cameras_with_pc_bbox(
        typename pcl::PointCloud<pT>::Ptr pPC,
        const std::vector< CameraProjection<rT> > &inCamProjs,
        std::vector< CameraProjection<rT> > &outCamProjs,
        const std::string &outDir="" ) {
    QUICK_TIME_START(te)

    std::cout << inCamProjs.size() << " cameras to filter against bbox of the input point cloud. \n";

    outCamProjs.clear();

    // Compute the bounding box of the input point cloud.
    pcu::OBB<pT, rT> obb;
    pcu::get_obb<pT, rT>(pPC, obb);

    // Test use.
    std::cout << "obb: \n" << obb << "\n";

    // Find the 8 corners from the bbox.
    Eigen::MatrixX<rT> points;
    find_corner_points_from_bbox<pT, rT>( obb.minPoint, obb.maxPoint, points );

    // Transform the corner points to the world frame.
    Eigen::Vector3<rT> position;
    position << obb.position.x, obb.position.y, obb.position.z;

    points = obb.rotMat * points.eval();
    points.colwise() += position;

    // Test use.
    std::cout << "points = \n" << points << "\n";
    if ( !outDir.empty() ) {
        std::string outFn = outDir + "/BBoxPointsForFilteringCameras.csv";
        write_matrix(outFn, points.transpose());
        std::cout << "Saved " << outFn << ". \n";
    }

    // Loop over all the cameras.
    QUICK_TIME_START(teLoop)
    for ( const auto& cp : inCamProjs ) {
        if ( !cp.are_world_points_outside_frustum(points) ) {
            outCamProjs.emplace_back( cp );
        }
    }
    QUICK_TIME_END(teLoop)
    std::cout << "Loop in " << teLoop << " ms. \n";

    if ( outCamProjs.empty() ) {
        BOOST_THROW_EXCEPTION( NoCameraFound()
            << ExceptionInfoString("No camera found from filtering by the bbox of the input point cloud.") );
    }

    QUICK_TIME_END(te)
    std::cout << "Filter cameras against point cloud's bbox in " << te << " ms. "
              << outCamProjs.size() << " cameras remain. \n";
}

template < typename rT >
static void write_cameras( const std::string &fn,
                           const std::vector<CameraProjection<rT>> &camProjs ) {
    std::ofstream ofs(fn);

    if ( !ofs.good() ) {
        EXCEPTION_FILE_NOT_GOOD(fn)
    }

    const int N = camProjs.size();
    assert( N > 0 );

    const std::string TAB = "    ";

    ofs << "{" << "\n";
    ofs << "\"camProjs\": [" << "\n";

    for ( int i = 0; i < N; ++i ) {
        ofs << TAB;
        camProjs[i].write_json_content(ofs, TAB, 1);

        if ( i != N-1 ) {
            ofs << "," << "\n";
        } else {
            ofs << " ]" << "\n";
        }
    }

    ofs << "}" << "\n";

    ofs.close();
}

static CPs_t filter_cameras_against_bbox_of_point_cloud(
        const PCPtr_t pInCloud, const CPs_t &camProjOri,
        const std::string &outDir ){
    CPs_t camProj;
    filter_cameras_with_pc_bbox<P_t, float>(pInCloud, camProjOri, camProj, outDir);
    return camProj;
}

int main( int argc, char** argv ) {
    QUICK_TIME_START(teMain)
    std::cout << "Hello, FilterCamerasByBBox! \n";
    MAIN_COMMON_LINES_ONE_CLASS(argc, argv, args)

    // Prepare the output directory.
    test_directory(args.outDir);

    // Read the point cloud.
    print_bar("Read point cloud.");
    PCPtr_t pInCloud = pcu::read_point_cloud<P_t>( args.inCloud );

    // Read the camera pose file.
    print_bar("Read camera pose file.");
    CamPoseCSVRepresentation<float> cpCSVR;
    read_camera_poses_csv(args.inCamPoses, cpCSVR);

    // Convert the camera pose to PCL point cloud and save the point cloud.
    auto pCamZ = pcu::convert_camera_poses_2_pcl( cpCSVR.quat, cpCSVR.pos );
    {
        std::string outFn = args.outDir + "/CameraPoses.ply";
        pcu::write_point_cloud<pcl::PointNormal>(outFn, pCamZ);
    }

    // Camera intrinsics.
    print_bar("Read camera intrinsics.");
    auto camK = read_camera_intrinsics_from_p1(args.inCamP1);

    // Read the CameraProjection objects.
    print_bar("Read CameraProjection objects.");
    auto camProjOri = read_camera_projs( args.inImgSize, cpCSVR, camK );

    // Filter the cameras. Keep the cameras that can see the input point cloud's bounding box.
    print_bar("Filter cameras against the bbox of the input point cloud.");
    auto camProj = filter_cameras_against_bbox_of_point_cloud(pInCloud, camProjOri, args.outDir);

    {
        std::string outFn = args.outDir + "/FilteredCamProjBBox.json";
        write_cameras(outFn, camProj);
    }

    QUICK_TIME_SHOW(teMain, "FilterCameraByBBox")
    return 0;
}