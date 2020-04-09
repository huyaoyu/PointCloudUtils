//
// Created by yaoyu on 4/6/20.
//

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/exception/all.hpp>
#include <boost/program_options.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include "Args/Args.hpp"
#include "CVCommon/All.hpp"
#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "DataInterfaces/NumPy/IO.hpp"
#include "Filesystem/Filesystem.hpp"
#include "HoleBoundaryDetection/HoleBoundaryProjector.hpp"
#include "PCCommon/BBox.hpp"
#include "PCCommon/common.hpp"
#include "PCCommon/extraction.hpp"
#include "PCCommon/IO.hpp"

#include "Tools/CropByOBBox.hpp"

// Namespaces.
namespace bpo = boost::program_options;
using JSON = nlohmann::json;

#define EXCEPTION_INVALID_ARGUMENTS(args) \
    {\
        std::stringstream args##_ss;\
        args##_ss << "Arguments invalidation failed. " << std::endl \
                  << args << std::endl;\
        BOOST_THROW_EXCEPTION( args_invalidation_failed() << ExceptionInfoString(args##_ss.str()) );\
    }

#define EXCEPTION_ID_NOT_FOUND(id) \
    {\
        std::stringstream id##_ss;\
        id##_ss << "ID " \
                << id << " is not found.";\
        BOOST_THROW_EXCEPTION( id_not_found() << ExceptionInfoString(id##_ss.str()) );\
    }

#define EXCEPTION_FILE_NOT_GOOD(fn) \
    {\
        std::stringstream fn##_ss; \
        fn##_ss << "File " << fn << " is not good. "; \
        BOOST_THROW_EXCEPTION( file_not_good() << ExceptionInfoString(fn##_ss.str()) ); \
    }

#define EXCEPTION_DIAG_INFO(ex) \
    boost::diagnostic_information(ex)

// Exception definitions.
struct exception_base           : virtual std::exception, virtual boost::exception { };
struct args_invalidation_failed : virtual exception_base {};
struct id_not_found             : virtual exception_base {};
struct file_not_good            : virtual exception_base {};

typedef boost::error_info<struct tag_info_string, std::string> ExceptionInfoString;

class Args
{
public:
    Args() : holeID(-1) {}

    ~Args() = default;

    bool validate() {
        bool flag = true;

        if ( -1 == holeID ) {
            flag = false;
            std::cout << "Currently only support single hole processing. " << std::endl;
        }

        if ( camScale <= 0 ) {
            flag = false;
            std::cout << "Camera intrinsic scale must be positive. " << camScale << " is specified. " << std::endl;
        }

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_MVS_O << ": " << args.inMVSOriginal << std::endl;
        out << Args::AS_IN_MVS_B << ": " << args.inMVSBoundary << std::endl;
        out << Args::AS_IN_LIDAR << ": " << args.inLiDAR << std::endl;
        out << Args::AS_IN_HOLE_PROJ << ": " << args.inHoleProj << std::endl;
        out << Args::AS_HOLE_ID << ": " << args.holeID << std::endl;
        out << Args::AS_OUR_DIR << ": " << args.outDir << std::endl;
        out << Args::AS_CAM_SCALE << ": " << args.camScale << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_MVS_O; // AS stands for argument string
    static const std::string AS_IN_MVS_B;
    static const std::string AS_IN_LIDAR;
    static const std::string AS_IN_HOLE_PROJ;
    static const std::string AS_HOLE_ID;
    static const std::string AS_OUR_DIR;
    static const std::string AS_CAM_SCALE;

public:
    std::string inMVSOriginal; // The MVS point cloud on which we will be perform hole depth completion.
    std::string inMVSBoundary; // The MVS point cloud on which we have performed hole boundary identification.
    std::string inLiDAR; // The reference point cloud.
    std::string inHoleProj; // The JSON file that records the hole projections.
    int holeID; // The hole id to process.
    std::string outDir; // The output directory.
    float camScale; // The scale of the camera intrinsics.
};

const std::string Args::AS_IN_MVS_O = "inMVSOriginal";
const std::string Args::AS_IN_MVS_B = "inMVSBoundary";
const std::string Args::AS_IN_LIDAR = "inLiDAR";
const std::string Args::AS_IN_HOLE_PROJ = "inHoleProj";
const std::string Args::AS_HOLE_ID = "holeID";
const std::string Args::AS_OUR_DIR = "outDir";
const std::string Args::AS_CAM_SCALE = "cam-scale";

static void parse_args(int argc, char* argv[], Args& args) {

    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_MVS_O.c_str(), bpo::value< std::string >(&args.inMVSOriginal)->required(), "The input MVS point cloud for hole completion.")
                (Args::AS_IN_MVS_B.c_str(), bpo::value< std::string >(&args.inMVSBoundary)->required(), "The input MVS point cloud for hole identification.")
                (Args::AS_IN_LIDAR.c_str(), bpo::value< std::string >(&args.inLiDAR)->required(), "The input LiDAR point cloud.")
                (Args::AS_IN_HOLE_PROJ.c_str(), bpo::value< std::string >(&args.inHoleProj)->required(), "The input JSON file that records the hole projection.")
                (Args::AS_HOLE_ID.c_str(), bpo::value< int >(&args.holeID)->default_value(-1), "The single hole ID that needs to be processed. Use -1 of leave unset to process all hole IDs." )
                (Args::AS_OUR_DIR.c_str(), bpo::value< std::string >(&args.outDir)->required(), "The output file.")
                (Args::AS_CAM_SCALE.c_str(), bpo::value< float >(&args.camScale)->default_value(1.0f), "The scale of the camera intrinsics. Positive floating point number.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(Args::AS_IN_MVS_O.c_str(), 1
        ).add(Args::AS_IN_MVS_B.c_str(), 1
        ).add(Args::AS_IN_LIDAR.c_str(), 1
        ).add(Args::AS_IN_HOLE_PROJ.c_str(), 1
        ).add(Args::AS_OUR_DIR.c_str(), 1);

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

    if ( !args.validate() ) {
        EXCEPTION_INVALID_ARGUMENTS(args)
    }
}

static std::shared_ptr<JSON> read_json( const std::string &fn ) {
    std::shared_ptr<JSON> pJson ( new JSON );

    std::ifstream ifs(fn);

    if ( !ifs.good() ) {
        EXCEPTION_FILE_NOT_GOOD(fn)
    }

    ifs >> *pJson;

    return pJson;
}

template < typename rT, typename derived >
static void convert_vector_2_eigen_vector( const std::vector<rT> &v,
        Eigen::MatrixBase<derived> &ev ) {
    const std::size_t N = v.size();
    assert(N > 0);

    for ( std::size_t i = 0; i < N; ++i ) {
        ev(i) = v[i];
    }
}

template < typename rT >
static void convert_vector_2_eigen_mat3( const std::vector<rT> &v,
        Eigen::Matrix3<rT> &mat ) {
    mat << v[0], v[1], v[2],
           v[3], v[4], v[5],
           v[6], v[7], v[8];
}

template < typename rT >
static std::shared_ptr< pcu::HoleBoundaryPoints<rT> > find_boundary_polygon(
        const JSON &json, const int id ) {

    const auto& hpp = json["hpp"];

    const int N = hpp.size();

    int idFound = -1;

    // Try to find the id.
    for ( int i = 0; i < N; ++i ) {
        if ( id == hpp[i]["id"] ) {
            idFound = i;
            break;
        }
    }

    // Check if we find the id.
    if ( -1 == idFound ) {
        EXCEPTION_ID_NOT_FOUND(id)
    }

    // id found.
    const auto& hbp = hpp[idFound];

    std::shared_ptr< pcu::HoleBoundaryPoints<rT> > pHBP (new pcu::HoleBoundaryPoints<rT>);

    // Fill in the values.
    // id.
    pHBP->id = id;

    // polygonIndices.
    pHBP->polygonIndices.resize( hbp["polygonIndices"].size() );
    std::copy( hbp["polygonIndices"].begin(), hbp["polygonIndices"].end(), pHBP->polygonIndices.begin() );

    // equivalentNormal.
    pcl::PointNormal pn;
    pn.x = hbp["centroid"][0];
    pn.y = hbp["centroid"][1];
    pn.z = hbp["centroid"][2];
    pn.normal_x = hbp["normal"][0];
    pn.normal_y = hbp["normal"][1];
    pn.normal_z = hbp["normal"][2];
    pn.curvature = hbp["curvature"];
    pHBP->equivalentNormal = pn;

    // CameraProjection.
    CameraProjection<rT> cp;
    const auto& camProj = hbp["camProj"];

    cp.id     = camProj["id"];
    cp.height = camProj["height"];
    cp.width  = camProj["width"];

    convert_vector_2_eigen_mat3( camProj["K"].get< std::vector<float> >(), cp.K );
    convert_vector_2_eigen_mat3( camProj["RC"].get< std::vector<float> >(), cp.RC );
    convert_vector_2_eigen_mat3( camProj["R"].get< std::vector<float> >(), cp.R );
    convert_vector_2_eigen_vector( camProj["T"].get< std::vector<rT> >(), cp.T );

//    Eigen::Vector4<rT> qv;
//    convert_vector_2_eigen_vector( camProj["T"].get< std::vector<rT> >(), qv );

    std::vector<rT> qv = camProj["Q"].get< std::vector<rT> >();

    cp.Q = Eigen::Quaternion<rT>( qv[0], qv[1], qv[2], qv[3] );

    pHBP->camProj = cp;

    return pHBP;
}

template < typename pT, typename rT >
static void project_pcl_points_2_pixel_plane(
        const typename pcl::PointCloud<pT>::Ptr pInput,
        const CameraProjection<rT> &camProj,
        Eigen::MatrixX<rT> &pixels ) {

    const std::size_t N = pInput->size();
    assert(N > 0);

    pixels.resize(3, N);
//    pixels = Eigen::Matrix<rT, 3, N>::Ones();

    for ( std::size_t i = 0; i < N; ++i ) {
        const auto& point = pInput->at(i);

        Eigen::Vector3<rT> wp;
        wp << point.x, point.y, point.z;

        Eigen::Vector3<rT> pixel = Eigen::Vector3<rT>::Ones();
        camProj.world_2_pixel(wp, pixel);

        pixels.col(i) = pixel;
    }
}

/**
 * Find the 2D bounding box parallel to the axes in the pixel plane.
 * @tparam rT A floating point type.
 * @param pixels The matrix stores the pixels coordinates. 3xn matrix, the last row should be all 1.0.
 * @param x0 X coordinate of the upper-left corner of the bounding box.
 * @param y0 Y coordinate of the upper-left corner of the bounding box.
 * @param x1 X coordinate of the lower-right corner of the bounding box.
 * @param y1 Y coordinate of the lower-right corner of the bounding box.
 */
template < typename rT >
static void find_bounding_box_pixel_plane( const Eigen::MatrixX<rT> &pixels,
        rT &x0, rT &y0, rT &x1, rT &y1 ) {
    const std::size_t N = pixels.cols();

    assert( N > 0 );

    x0 = pixels(0,0);
    y0 = pixels(1,0);
    x1 = x0;
    y1 = y0;

    rT x, y;

    for ( std::size_t i = 0; i < N; ++i ) {
        x = pixels(0, i);
        y = pixels(1, i);

        if ( x < x0 ) {
            x0 = x;
        } else if ( x > x1 ) {
            x1 = x;
        }

        if ( y < y0 ) {
            y0 = y;
        } else if ( y > y1 ) {
            y1 = y;
        }
    }
}

template < typename pT >
static void shift_bbox_borders_metric( pT &minPoint, pT &maxPoint, float s ) {
    minPoint.x -= s;
    minPoint.y -= s;
    minPoint.z -= s;

    maxPoint.x += s;
    maxPoint.y += s;
    maxPoint.z += s;
}

template < typename pT, typename rT >
static void make_depth_map(
        const typename pcl::PointCloud<pT>::Ptr pInput,
        const CameraProjection<rT> &camProj,
        Eigen::MatrixX<rT> &depth ) {
    QUICK_TIME_START(te)

    // Save all the points to an Eigen matrix.
    Eigen::MatrixX<rT> points;
    pcu::convert_pcl_2_eigen_matrix<pT, rT>( pInput, points );

    // Transform all the points into the camera(sensor) frame.
    Eigen::MatrixX<rT> cp =
            camProj.RC.transpose() * camProj.R.transpose() * ( points.colwise() - camProj.T );

    // Find all the points that have positive z-coordinate.
    const std::size_t cpRows = cp.rows();
    const std::size_t cpCols = cp.cols();
    Eigen::MatrixX<rT> cpz( cpRows, cpCols );

    std::size_t count = 0;

    for ( std::size_t j = 0; j < cpCols; ++j ) {
        if ( cp(2, j) <= 0 ) {
            continue;
        }

        cpz( Eigen::all, count) = cp( Eigen::all, j );
        count++;
    }

    // Test use.
    std::cout << count << " points transformed to the positive-z positions. " << std::endl;

    // Copy to an Eigen matrix with the right dimension.
    Eigen::MatrixX<rT> cpp = cpz( Eigen::all, Eigen::seq(0, count-1) );

//    // Test use.
//    Eigen::MatrixX<rT> wp = ( camProj.R * camProj.RC * cpp ).colwise() + camProj.T;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pWP =
//            pcu::convert_eigen_matrix_2_pcl_xyz<rT>( wp );
//    pcu::write_point_cloud<pcl::PointXYZ>("./WP.ply", pWP);

    // Project cpp to the pixel plane.
    Eigen::MatrixX<rT> pixels = camProj.K * cpp;
    pixels = ( pixels.array().rowwise() / pixels(2, Eigen::all).array().eval() ).matrix();

//    // Test use.
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pPixels =
//            pcu::convert_eigen_matrix_2_pcl_xyz<rT>( pixels );
//    pcu::write_point_cloud<pcl::PointXYZ>("./Pixels.ply", pPixels);

    // Create the depth map.
    depth.resize( camProj.height, camProj.width );
    depth = Eigen::MatrixX<rT>::Constant(camProj.height, camProj.width, static_cast<rT>(-1.0));

    // Loop all the points in cpp;
    const std::size_t N = cpp.cols();
    std::size_t countDepth = 0;
    for ( std::size_t i = 0; i < N; ++i ) {
        const int x = static_cast<int>( std::round( pixels(0, i) ) );
        const int y = static_cast<int>( std::round( pixels(1, i) ) );

        if ( x < 0 || x >= camProj.width ||
             y < 0 || y >= camProj.height ) {
            continue;
        }

        const rT dInMap = depth(y, x);
        const rT d = cpp(2, i);

        if ( dInMap < 0 || d < dInMap ) {
            depth(y, x) = d;
            countDepth++;
        }
    }

    std::cout << countDepth << " depth point projected to the pixel plane. " << std::endl;

//    // Test use.
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pDepth =
//            pcu::convert_eigen_depth_img_2_pcl_xyz<rT>( depth );
//    pcu::write_point_cloud<pcl::PointXYZ>("./Depth.ply", pDepth);

    QUICK_TIME_END(te)

    std::cout << "Make depth map in " << te << " ms. " << std::endl;
}

template < typename pT, typename rT >
static void write_cropped_depth_info(
        const typename pcl::PointCloud<pT>::Ptr pInput,
        const pcl::PointXYZ &obbMinPoint,
        const pcl::PointXYZ &obbMaxPoint,
        const pcl::PointXYZ &obbPosition,
        const Eigen::Matrix3<rT> &obbRotMat,
        const CameraProjection<rT> &camProj,
        const std::string &outDir,
        const std::string &prefix ) {
    // Crop out the points from MVSO in the oriented bounding box.
    typename pcl::PointCloud<pT>::Ptr pCropped =
            pcu::crop_by_oriented_bbox<pT, rT>( pInput,
                    obbMinPoint, obbMaxPoint, obbPosition, obbRotMat );

    // Make a depth map for the MVSO.
    Eigen::MatrixX<rT> depthCropped;
    make_depth_map<pT, rT>( pCropped, camProj, depthCropped );

    // Save the matrix as a float image.
    std::string mvsoDepthImgFn = outDir + "/" + prefix + "_DepthImg.png";
    write_eigen_matrixXf_2_image( mvsoDepthImgFn, depthCropped, 0.0f, 1.0f );

    // Save the depth map as a table as NumPy format for other processing.
    std::string depthNpyFn = outDir + "/" + prefix + "_DepthTable.npy";
    write_depth_map_2_npy(depthNpyFn, depthCropped);

    // Save the cropped point cloud for other processing.
    std::string croppedFn = outDir + "/" + prefix + "_Cropped.ply";
    pcu::write_point_cloud<pT>( croppedFn, pCropped );
}

template < typename rT >
static void write_json( const std::string &fn,
        int boundaryId,
        rT camScale,
        const CameraProjection<rT> &camProj ) {
    std::ofstream ofs(fn);

    if ( !ofs.good() ) {
        EXCEPTION_FILE_NOT_GOOD(fn)
    }

    const std::string TAB = "    ";

    ofs << "{" << std::endl;

    ofs << TAB << "\"boundaryId\": " << boundaryId << "," << std::endl;
    ofs << TAB << "\"camScale\": " << camScale << "," << std::endl;

    ofs << TAB << "\"camProj\": ";
    camProj.write_json_content(ofs, TAB, 1);
    ofs << std::endl << "}";
}

int main( int argc, char* argv[] ) {
    std::cout << "Hello, HoleDepthCompletionDataGeneration! " << std::endl;

    // Handle the command line.
    Args args;
    parse_args(argc, argv, args);

    std::cout << "args: " << std::endl;
    std::cout << args << std::endl;

    // The output directory with the hole ID.
    std::stringstream ssOutDir;
    ssOutDir << args.outDir << "/" << args.holeID;
    std::string outDir = ssOutDir.str();

    // Test the output directory.
    test_directory(outDir);

    // Read the hole boundary projection/polygon JSON file,
    // find and create a HoleBoundaryPoints object according to the specified ID.
    std::shared_ptr<JSON> pJson = read_json( args.inHoleProj );

    auto pHBP = std::make_shared<pcu::HoleBoundaryPoints<float>>( );

    try {
        pHBP = find_boundary_polygon<float>( *pJson, args.holeID );
    } catch ( id_not_found &ex ) {
        std::cout << EXCEPTION_DIAG_INFO(ex) << std::endl;
        std::cout << "ID " << args.holeID << " not found. Abort! " << std::endl;
        return 1;
    } catch ( ... ) {
        std::cout << "Unknown error! " << std::endl;
        return 1;
    }

    // Test use.
    std::cout << "pHBP: " << std::endl;
    std::cout << *pHBP << std::endl;

    // Make a copy of the CameraProjection object.
    CameraProjection<float> scaledCamProj( pHBP->camProj );

    // Scale the camera intrinsics.
    scaledCamProj.scale_intrinsics(args.camScale);

    // Test use.
    std::cout << "scaledCampProj = " << std::endl;
    std::cout << scaledCamProj << std::endl;

    // Load MVSB.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pMVSB =
            pcu::read_point_cloud<pcl::PointXYZ>( args.inMVSBoundary );

    // Extract the boundary points.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pBoundary =
            pcu::extract_points<pcl::PointXYZ, int>( pMVSB, pHBP->polygonIndices );

    // Save the boundary points for future process.
    std::string boundaryPointsFn = outDir + "/BoundaryPoints.ply";
    pcu::write_point_cloud<pcl::PointXYZ>( boundaryPointsFn, pBoundary );

    // Project the boundary points to the 2D pixel plane.
    Eigen::MatrixXf boundaryPixels;
    project_pcl_points_2_pixel_plane<pcl::PointXYZ, float>(
            pBoundary, scaledCamProj, boundaryPixels );

    // Find the 2D bounding box in the pixel plane.
    float bx0, by0, bx1, by1;
    find_bounding_box_pixel_plane( boundaryPixels, bx0, by0, bx1, by1 );

    // Save the boundary pixels.
    std::string boundaryPixelFn = outDir + "/BoundaryPixels.npy";
    write_eigen_matrix_2_npy( boundaryPixelFn, boundaryPixels);

    // Test use.
    std::cout << "Pixel bounding box coordinates: "
              << "( " << bx0 << ", " << by0 << " ), "
              << "( " << bx1 << ", " << by1 << " ). " << std::endl;

    // Compute the oriented bounding box of the boundary points.
    pcl::PointXYZ obbMinPoint, obbMaxPoint, obbPosition;
    Eigen::Matrix3f obbRotMat;
    pcu::get_obb<pcl::PointXYZ>(pBoundary, obbMinPoint, obbMaxPoint, obbPosition, obbRotMat);

    // Enlarge the b-box a little bit.
    shift_bbox_borders_metric( obbMinPoint, obbMaxPoint, 0.02 );

    std::vector<std::string> inputCloudFn = { args.inMVSOriginal, args.inLiDAR };
    std::vector<std::string> outputPrefix = { "MVSO", "LiDAR" };

    const int N = inputCloudFn.size();

    for ( int i = 0; i < N; ++i ) {
        std::cout << "Process " << inputCloudFn[i] << std::endl;

        // Load point cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud =
                pcu::read_point_cloud<pcl::PointXYZ>( inputCloudFn[i] );

        // Write depth info.
        write_cropped_depth_info<pcl::PointXYZ, float>(
                pPointCloud,
                obbMinPoint, obbMaxPoint, obbPosition, obbRotMat,
                scaledCamProj, outDir, outputPrefix[i] );
    }

    // Write the JSON file.
    std::string jsonFn = outDir + "/DepthCompletion.json";
    write_json( jsonFn, pHBP->id, args.camScale, pHBP->camProj );

    return 0;
}