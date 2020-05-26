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
#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "DataInterfaces/Plain/Matrix.hpp"
#include "Exception/Common.hpp"
#include "Filesystem/Filesystem.hpp"
#include "HoleBoundaryDetection/HoleBoundaryDetector.hpp"
#include "HoleBoundaryDetection/HoleBoundaryProjector.hpp"
#include "OccupancyMap/OccupancyMap.hpp"
#include "PCCommon/IO.hpp"
#include "PCCommon/BBox.hpp"
#include "Visualization/Print.hpp"

// Namespaces.
namespace bpo = boost::program_options;

struct NoCameraFound : virtual exception_common_base {};

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
        out << Args::AS_IN_CLOUD << ": " << args.inCloud << "\n";
        out << Args::AS_IN_OCP_MAP << ": " << args.inOcpMap << "\n";
        out << Args::AS_IN_SETS_JSON << ": " << args.inJson << "\n";
        out << Args::AS_IN_CAM_POSES << ": " << args.inCamPoses << "\n";
        out << Args::AS_IN_CAM_P1 << ": " << args.inCamP1 << "\n";
        out << Args::AS_IN_IMG_SIZE << ": " << args.inImgSize << "\n";
        out << Args::AS_OUT_DIR << ": " << args.outDir << "\n";
        out << Args::AS_PROJ_NUM_LIM << ": " << args.projNumLimit << "\n";
        out << Args::AS_PROJ_CURV_LIM << ": " << args.projCurvLimit << "\n";

        return out;
    }

public:
    static const std::string AS_IN_CLOUD; // AS stands for argument string
    static const std::string AS_IN_OCP_MAP; // The occupancy map.
    static const std::string AS_IN_SETS_JSON;
    static const std::string AS_IN_CAM_POSES;
    static const std::string AS_IN_CAM_P1;
    static const std::string AS_IN_IMG_SIZE;
    static const std::string AS_OUT_DIR;
    static const std::string AS_PROJ_NUM_LIM;
    static const std::string AS_PROJ_CURV_LIM;

public:
    std::string inCloud; // The input point cloud file.
    std::string inOcpMap; // The occupancy map. With free and occupied voxels defined.
    std::string inJson;
    std::string inCamPoses; // The input camera poses.
    std::string inCamP1; // The camera intrinsics, 3x4 matrix. The 3x3 part is K.
    std::string inImgSize; // The JSON file contains the image size.
    std::string outDir; // The output directory.
    int projNumLimit;
    float projCurvLimit; // The maximum curvature value for a point set to be considered as flat.
};

const std::string Args::AS_IN_CLOUD      = "in-cloud";
const std::string Args::AS_IN_OCP_MAP    = "in-ocp-map";
const std::string Args::AS_IN_SETS_JSON  = "in-json";
const std::string Args::AS_IN_CAM_POSES  = "in-cam-poses";
const std::string Args::AS_IN_CAM_P1     = "in-cam-p1";
const std::string Args::AS_IN_IMG_SIZE   = "in-img-size";
const std::string Args::AS_OUT_DIR       = "outdir";
const std::string Args::AS_PROJ_NUM_LIM  = "proj-num-limit";
const std::string Args::AS_PROJ_CURV_LIM = "proj-curv-limit";

static void parse_args(int argc, char* argv[], Args& args) {
    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_CLOUD.c_str(), bpo::value< std::string >(&args.inCloud)->required(), "Input point cloud.")
                (Args::AS_IN_OCP_MAP.c_str(), bpo::value< std::string >(&args.inOcpMap)->required(), "The occupancy map.")
                (Args::AS_IN_SETS_JSON.c_str(), bpo::value< std::string >(&args.inJson)->required(), "The JSON file stores the disjoint sets.")
                (Args::AS_IN_CAM_POSES.c_str(), bpo::value< std::string >(&args.inCamPoses)->required(), "The camera pose CSV file.")
                (Args::AS_IN_CAM_P1.c_str(), bpo::value< std::string >(&args.inCamP1)->required(), "The P1 matrix.")
                (Args::AS_IN_IMG_SIZE.c_str(), bpo::value< std::string >(&args.inImgSize)->required(), "The image size JSON file.")
                (Args::AS_OUT_DIR.c_str(), bpo::value< std::string >(&args.outDir)->required(), "Output directory.")
                (Args::AS_PROJ_NUM_LIM.c_str(), bpo::value< int >(&args.projNumLimit)->default_value(3), "The limit number of points for a projection.")
                (Args::AS_PROJ_CURV_LIM.c_str(), bpo::value< float >(&args.projCurvLimit)->default_value(0.01), "The curvature limit for a point set to be considered as flat");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(
                Args::AS_IN_CLOUD.c_str(), 1
                ).add(Args::AS_IN_OCP_MAP.c_str(), 1
                ).add(Args::AS_IN_SETS_JSON.c_str(), 1
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

    if ( !args.validate() ) {
        EXCEPTION_INVALID_ARGUMENTS(args)
    }
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

static void write_hole_polygon_points_json(
        const std::string &fn,
        const std::vector<pcu::HoleBoundaryPoints<float>> &hpp ) {

    std::ofstream ofs(fn);

    if ( !ofs.good() ) {
        EXCEPTION_FILE_NOT_GOOD(fn)
    }

    const int N = hpp.size();
    const std::string TAB = "    ";

    ofs << "{" << "\n";
    ofs << "\"hpp\": [" << "\n";

    for ( int i = 0; i < N; ++i ) {
        ofs << TAB;
        hpp[i].write_json_content(ofs, TAB, 1);

        if ( i != N-1 ) {
            ofs << "," << "\n";
        } else {
            ofs << " ]" << "\n";
        }
    }

    ofs << "}" << "\n";

    ofs.close();
}

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
    std::cout << "obb: \n";
    std::cout << obb << "\n";

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
    if ( outDir != "" ) {
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

    if ( 0 == outCamProjs.size() ) {
        BOOST_THROW_EXCEPTION( NoCameraFound()
            << ExceptionInfoString("No camera found from filtering by the bbox of the input point cloud.") );
    }

    QUICK_TIME_END(te)
    std::cout << "Filter cameras against point cloud's bbox in " << te << " ms. "
              << outCamProjs.size() << " cameras remain. \n";
}

struct BoundaryPointSets {
    BoundaryPointSets()
        : pEquivalentNormals( new pcl::PointCloud<pcl::PointNormal> )
    {}

    BoundaryPointSets( const BoundaryPointSets &other ) {
        *(this->pEquivalentNormals) = *(other.pEquivalentNormals);
        this->candidateSets = other.candidateSets;
    }

    BoundaryPointSets( BoundaryPointSets &&other ) {
        this->pEquivalentNormals.reset( other.pEquivalentNormals.get() );
        other.pEquivalentNormals.reset( new pcl::PointCloud<pcl::PointNormal> );

        this->candidateSets = std::move( other.candidateSets );
    }

    BoundaryPointSets& operator = ( const BoundaryPointSets &other ) {
        if ( this == &other ) {
            return *this;
        }

        *(this->pEquivalentNormals) = *(other.pEquivalentNormals);
        this->candidateSets = other.candidateSets;

        return *this;
    }

    BoundaryPointSets& operator = ( BoundaryPointSets &&other ) {
        if ( this == &other ) {
            return *this;
        }

        this->pEquivalentNormals.reset( other.pEquivalentNormals.get() );
        other.pEquivalentNormals.reset( new pcl::PointCloud<pcl::PointNormal> );

        this->candidateSets = std::move( other.candidateSets );

        return *this;
    }

    ~BoundaryPointSets() = default;

    bool check_size() const {
        return pEquivalentNormals->size() == candidateSets.size();
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr pEquivalentNormals;
    std::vector< std::vector<int> > candidateSets;
};

static bool check_possible_occlusion(
        const pcl::PointCloud<pcl::PointNormal>::Ptr pPC,
        const std::vector<int> &indices,
        const pcu::OccupancyMap &ocpMap,
        float threshold=0.33 ) {
    assert( threshold > 0 );

    const int nIndices = indices.size();
    int count = 0; // The number of occluded points.
    bool flagOcclusion = false;

    for ( const auto& idx : indices ) {
        const auto& point = pPC->at(idx);

        if ( ocpMap.check_near_frontier( point.x, point.y, point.z, 2 ) ) {
            count++;

            if ( static_cast<float>(count) / nIndices >= threshold ) {
                flagOcclusion = true;
                break;
            }
        }
    }

    return flagOcclusion;
}

static BoundaryPointSets filter_boundary_point_sets_by_occupancy_map(
        const pcl::PointCloud<pcl::PointNormal>::Ptr pPC,
        const BoundaryPointSets &boundaryPointSets,
        const pcu::OccupancyMap &ocpMap ) {
    QUICK_TIME_START(te)
    assert( boundaryPointSets.check_size() );

    BoundaryPointSets filteredBPS;

    const int N = boundaryPointSets.candidateSets.size();

    for ( int i = 0; i < N; ++i ) {
        const std::vector<int> &indices = boundaryPointSets.candidateSets[i];
        if ( !check_possible_occlusion(pPC, indices, ocpMap) ) {
            filteredBPS.pEquivalentNormals->push_back(
                    boundaryPointSets.pEquivalentNormals->at(i) );
            filteredBPS.candidateSets.emplace_back( indices );
        }
    }

    std::cout << filteredBPS.candidateSets.size() << " / " << N
              << " boundary candidate point sets remain after filtering by the occupancy map. \n";

    QUICK_TIME_SHOW(te, "Filter boundary point sets by the occupancy map")

    return filteredBPS;
}

int main(int argc, char* argv[]) {
    std::cout << "Hello, HoleBoundaryProjection! \n";

    // Handle the command line.
    MAIN_COMMON_LINES(argc, argv, args)

    test_directory(args.outDir);

    print_bar("Read point cloud, disjoint boundary candidates, and camera pose files.");
    // Read the point cloud.
    pcl::PointCloud<pcl::PointNormal>::Ptr pInCloud ( new pcl::PointCloud<pcl::PointNormal> );
    pcu::read_point_cloud<pcl::PointNormal>( args.inCloud, pInCloud );

    // Read the image size JSON file.
    float imgHeight, imgWidth;
    read_image_size_json( args.inImgSize, imgHeight, imgWidth );
    std::cout << "Image size is ( " << imgHeight << ", " << imgWidth << " ). \n";

    // Read the JSON file.
    BoundaryPointSets boundaryPointSets;
    pcu::read_equivalent_normal_from_json( args.inJson,
            boundaryPointSets.pEquivalentNormals,
            boundaryPointSets.candidateSets );

    // Read the occupancy map.
    pcu::OccupancyMap ocpMap;
    ocpMap.read(args.inOcpMap);

    std::cout << "Read the occupancy map. "
              << ocpMap.num_frontiers() << " frontier voxels. \n";

    // Filter the equivalent normals and boundary candidate points by the occupancy map.
    std::cout << "Begin filter the boundary candidate point sets. \n";
    BoundaryPointSets filteredBPS =
            filter_boundary_point_sets_by_occupancy_map(
                    pInCloud, boundaryPointSets, ocpMap );

//    // Test use.
//    throw std::runtime_error("Test");

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

    std::vector< CameraProjection<float> > camProjOri;
    convert_from_quaternion_translation_table( camID, camQuat, camPos, camK, camProjOri );
    set_image_size_for_camera_projection_objects( camProjOri, imgHeight, imgWidth );

    // Filter the cameras. Keep the cameras that can see the input point cloud's bounding box.
    print_bar("Filter cameras against the bbox of the input point cloud.");
    std::vector< CameraProjection<float> > camProj;
    filter_cameras_with_pc_bbox<pcl::PointNormal, float>(pInCloud, camProjOri, camProj, args.outDir);
    {
        std::string outFn = args.outDir + "/FilteredCamProjBBox.json";
        write_cameras(outFn, camProj);
    }

//    // Test use.
//    std::cout << "camProj.size() = " << camProj.size() << "\n";
//    std::cout << "camProj[ camProj.siz()-1 ].T = " << "\n" << camProj[ camProj.size()-1 ].T << "\n";
//    std::cout << "camProj[ camProj.siz()-1 ].K = " << "\n" << camProj[ camProj.size()-1 ].K << "\n";
    std::cout << camProj[0] << "\n";

    print_bar("Project the boundary candidates.");
    // Hole boundary projector.
    pcu::HoleBoundaryProjector<pcl::PointNormal, float> hbp;
    hbp.set_projection_number_limit(args.projNumLimit);
    hbp.set_projection_curvature_limit(args.projCurvLimit);

    std::vector< pcu::HoleBoundaryPoints<float> > holePolygonPoints;

    hbp.process( pInCloud,
            filteredBPS.candidateSets, filteredBPS.pEquivalentNormals,
            camProj, holePolygonPoints );

    {
        std::string outFn = args.outDir + "/HolePolygonPoints.json";
        write_hole_polygon_points_json(outFn, holePolygonPoints);
    }

    return 0;
}