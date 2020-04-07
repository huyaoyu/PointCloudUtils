//
// Created by yaoyu on 4/6/20.
//

#include <algorithm>
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

#include "Args/Args.hpp"
#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "Filesystem/Filesystem.hpp"
#include "HoleBoundaryDetection/HoleBoundaryProjector.hpp"
#include "PCCommon/IO.hpp"

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

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_MVS_O << ": " << args.inMVSOriginal << std::endl;
        out << Args::AS_IN_MVS_B << ": " << args.inMVSBoundary << std::endl;
        out << Args::AS_IN_LIDAR << ": " << args.inLiDAR << std::endl;
        out << Args::AS_IN_HOLE_PROJ << ": " << args.inHoleProj << std::endl;
        out << Args::AS_HOLE_ID << ": " << args.holeID << std::endl;
        out << Args::AS_OUR_DIR << ": " << args.outDir << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_MVS_O; // AS stands for argument string
    static const std::string AS_IN_MVS_B;
    static const std::string AS_IN_LIDAR;
    static const std::string AS_IN_HOLE_PROJ;
    static const std::string AS_HOLE_ID;
    static const std::string AS_OUR_DIR;

public:
    std::string inMVSOriginal; // The MVS point cloud on which we will be perform hole depth completion.
    std::string inMVSBoundary; // The MVS point cloud on which we have performed hole boundary identification.
    std::string inLiDAR; // The reference point cloud.
    std::string inHoleProj; // The JSON file that records the hole projections.
    int holeID; // The hole id to process.
    std::string outDir; // The output directory.
};

const std::string Args::AS_IN_MVS_O = "inMVSOriginal";
const std::string Args::AS_IN_MVS_B = "inMVSBoundary";
const std::string Args::AS_IN_LIDAR = "inLiDAR";
const std::string Args::AS_IN_HOLE_PROJ = "inHoleProj";
const std::string Args::AS_HOLE_ID = "holeID";
const std::string Args::AS_OUR_DIR = "outDir";

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
                (Args::AS_OUR_DIR.c_str(), bpo::value< std::string >(&args.outDir)->required(), "The output file.");

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

int main( int argc, char* argv[] ) {
    std::cout << "Hello, HoleDepthCompletionDataGeneration! " << std::endl;

    // Handle the command line.
    Args args;
    parse_args(argc, argv, args);

    std::cout << "args: " << std::endl;
    std::cout << args << std::endl;

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

    return 0;
}