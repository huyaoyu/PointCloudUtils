//
// Created by yaoyu on 3/30/20.
//

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

#include "Args/Args.hpp"
#include "CameraGeometry/CameraPose2PCL.hpp"
#include "CameraGeometry/IO.hpp"
#include "DataInterfaces/Plain/Matrix.hpp"
#include "Exception/Common.hpp"
#include "Filesystem/Filesystem.hpp"
#include "HoleBoundaryDetection/HoleBoundaryDetector.hpp"
#include "HoleBoundaryDetection/HoleBoundaryProjector.hpp"
#include "PCCommon/IO.hpp"
#include "Visualization/Print.hpp"

// Namespaces.
namespace bpo = boost::program_options;

class Args
{
public:
    Args(): projNumLimit(3) {}

    ~Args() = default;

    bool validate() {
        bool flag = true;

        if (projNumLimit < 3 ) {
            flag = false;
        }

        if ( projCurvLimit < 0 ) {
            flag = false;
        }

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_CLOUD << ": " << args.inCloud << std::endl;
        out << Args::AS_IN_SETS_JSON << ": " << args.inJson << std::endl;
        out << Args::AS_IN_CAM_POSES << ": " << args.inCamPoses << std::endl;
        out << Args::AS_IN_CAM_P1 << ": " << args.inCamP1 << std::endl;
        out << Args::AS_OUT_DIR << ": " << args.outDir << std::endl;
        out << Args::AS_PROJ_NUM_LIM << ": " << args.projNumLimit << std::endl;
        out << Args::AS_PROJ_CURV_LIM << ": " << args.projCurvLimit << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_CLOUD; // AS stands for argument string
    static const std::string AS_IN_SETS_JSON;
    static const std::string AS_IN_CAM_POSES;
    static const std::string AS_IN_CAM_P1;
    static const std::string AS_OUT_DIR;
    static const std::string AS_PROJ_NUM_LIM;
    static const std::string AS_PROJ_CURV_LIM;

public:
    std::string inCloud; // The input point cloud file.
    std::string inJson;
    std::string inCamPoses; // The input camera poses.
    std::string inCamP1; // The camera intrinsics, 3x4 matrix. The 3x3 part is K.
    std::string outDir; // The output directory.
    int projNumLimit;
    float projCurvLimit; // The maximum curvature value for a point set to be considered as flat.
};

const std::string Args::AS_IN_CLOUD      = "incloud";
const std::string Args::AS_IN_SETS_JSON  = "injson";
const std::string Args::AS_IN_CAM_POSES  = "incamposes";
const std::string Args::AS_IN_CAM_P1     = "incamp1";
const std::string Args::AS_OUT_DIR       = "outdir";
const std::string Args::AS_PROJ_NUM_LIM  = "proj-num-limit";
const std::string Args::AS_PROJ_CURV_LIM = "proj-curv-limit";

static void parse_args(int argc, char* argv[], Args& args) {
    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_CLOUD.c_str(), bpo::value< std::string >(&(args.inCloud))->required(), "Input point cloud.")
                (Args::AS_IN_SETS_JSON.c_str(), bpo::value< std::string >(&args.inJson)->required(), "The JSON file stores the disjoint sets.")
                (Args::AS_IN_CAM_POSES.c_str(), bpo::value< std::string >(&args.inCamPoses)->required(), "The camera pose CSV file.")
                (Args::AS_IN_CAM_P1.c_str(), bpo::value< std::string >(&args.inCamP1)->required(), "The P1 matrix.")
                (Args::AS_OUT_DIR.c_str(), bpo::value< std::string >(&args.outDir)->required(), "Output directory.")
                (Args::AS_PROJ_NUM_LIM.c_str(), bpo::value< int >(&args.projNumLimit)->default_value(3), "The limit number of points for a projection.")
                (Args::AS_PROJ_CURV_LIM.c_str(), bpo::value< float >(&args.projCurvLimit)->default_value(0.01), "The curvature limit for a point set to be considered as flat");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(
                Args::AS_IN_CLOUD.c_str(), 1
                ).add(Args::AS_IN_SETS_JSON.c_str(), 1
                ).add(Args::AS_IN_CAM_POSES.c_str(), 1
                ).add(Args::AS_IN_CAM_P1.c_str(), 1
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

    if ( !args.validate() ) {
        EXCEPTION_INVALID_ARGUMENTS(args)
    }
}

static void write_hole_polygon_points_json(
        const std::string &fn,
        const std::vector<pcu::HoleBoundaryPoints<float>> &hpp ) {

    std::ofstream ofs(fn);

    if ( !ofs.good() ) {
        std::stringstream ss;
        ss << fn << " is not good. ";
        throw( std::runtime_error(ss.str()) );
    }

    const int N = hpp.size();
    const std::string TAB = "    ";

    ofs << "{" << std::endl;
    ofs << "\"hpp\": [" << std::endl;

    for ( int i = 0; i < N; ++i ) {
        ofs << TAB;
        hpp[i].write_json_content(ofs, TAB, 1);

        if ( i != N-1 ) {
            ofs << "," << std::endl;
        } else {
            ofs << " ]" << std::endl;
        }
    }

    ofs << "}" << std::endl;

    ofs.close();
}

int main(int argc, char* argv[]) {
    std::cout << "Hello, HoleBoundaryProjection! " << std::endl;

    // Handle the command line.
    MAIN_COMMON_LINES(argc, argv, args)

    print_bar("Read point cloud, disjoint boundary candidates, and camera pose files.");
    // Read the point cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pInCloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcu::read_point_cloud<pcl::PointXYZRGB>( args.inCloud, pInCloud );

    // Read the JSON file.
    pcl::PointCloud<pcl::PointNormal>::Ptr pEquivalentNormals ( new pcl::PointCloud<pcl::PointNormal> );
    std::vector< std::vector<int> > candidateSets;
    pcu::read_equivalent_normal_from_json(args.inJson, pEquivalentNormals, candidateSets);

//    std::shared_ptr< std::vector< std::vector<int> > > pCandidateSets =
//            std::make_shared< std::vector< std::vector<int> > >(candidateSets);

    // Read the camera pose file.
    Eigen::VectorXi camID;
    Eigen::MatrixXf camQuat;
    Eigen::MatrixXf camPos;

    read_camera_poses_csv(args.inCamPoses, camID, camQuat, camPos);
    pcl::PointCloud<pcl::PointNormal>::Ptr pCamZ ( new pcl::PointCloud<pcl::PointNormal> );
    pcu::convert_camera_poses_2_pcl( camQuat, camPos, pCamZ );

    Eigen::MatrixXf camP1;
    read_matrix( args.inCamP1, 3, 4, " ", camP1 );
    Eigen::Matrix3f camK = camP1.block(0,0,3,3);

    std::vector< CameraProjection<float> > camProj;

    convert_from_quaternion_translation_table( camID, camQuat, camPos, camK, camProj );

//    // Test use.
//    std::cout << "camProj.size() = " << camProj.size() << std::endl;
//    std::cout << "camProj[ camProj.siz()-1 ].T = " << std::endl << camProj[ camProj.size()-1 ].T << std::endl;
//    std::cout << "camProj[ camProj.siz()-1 ].K = " << std::endl << camProj[ camProj.size()-1 ].K << std::endl;

    print_bar("Project the boundary candidates.");
    // Hole boundary projector.
    pcu::HoleBoundaryProjector<pcl::PointXYZRGB, float> hbp;
    hbp.set_projection_number_limit(args.projNumLimit);
    hbp.set_projection_curvature_limit(args.projCurvLimit);

    std::vector< pcu::HoleBoundaryPoints<float> > holePolygonPoints;

    hbp.process( pInCloud, candidateSets, pEquivalentNormals, camProj, holePolygonPoints );

    test_directory(args.outDir);

    {
        std::string outFn = args.outDir + "/HolePolygonPoints.json";
        write_hole_polygon_points_json(outFn, holePolygonPoints);
    }

    return 0;
}