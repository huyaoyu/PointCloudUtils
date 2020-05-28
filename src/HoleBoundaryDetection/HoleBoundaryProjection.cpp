//
// Created by yaoyu on 3/30/20.
//

#include <iostream>
#include <memory>
#include <string>

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
    Args(): projNumLimit(3), projCurvLimit(0.01) {}

    ~Args() = default;

    bool validate() const {
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
        out << Args::AS_IN_SETS_JSON << ": " << args.inSets << "\n";
        out << Args::AS_IN_CAM_PROJS << ": " << args.inCamProjs << "\n";
        out << Args::AS_OUT_DIR << ": " << args.outDir << "\n";
        out << Args::AS_PROJ_NUM_LIM << ": " << args.projNumLimit << "\n";
        out << Args::AS_PROJ_CURV_LIM << ": " << args.projCurvLimit << "\n";

        return out;
    }

public:
    static const std::string AS_IN_CLOUD; // AS stands for argument string
    static const std::string AS_IN_OCP_MAP; // The occupancy map.
    static const std::string AS_IN_SETS_JSON;
    static const std::string AS_IN_CAM_PROJS;
    static const std::string AS_OUT_DIR;
    static const std::string AS_PROJ_NUM_LIM;
    static const std::string AS_PROJ_CURV_LIM;

public:
    std::string inCloud; // The input point cloud file.
    std::string inOcpMap; // The occupancy map. With free and occupied voxels defined.
    std::string inSets;
    std::string inCamProjs; // The input camera poses.
    std::string outDir; // The output directory.
    int projNumLimit;
    float projCurvLimit; // The maximum curvature value for a point set to be considered as flat.
};

const std::string Args::AS_IN_CLOUD      = "in-cloud";
const std::string Args::AS_IN_OCP_MAP    = "in-ocp-map";
const std::string Args::AS_IN_SETS_JSON  = "in-sets";
const std::string Args::AS_IN_CAM_PROJS  = "in-cam-projs";
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
                (Args::AS_IN_SETS_JSON.c_str(), bpo::value< std::string >(&args.inSets)->required(), "The JSON file stores the disjoint sets.")
                (Args::AS_IN_CAM_PROJS.c_str(), bpo::value< std::string >(&args.inCamProjs)->required(), "The CameraProjection object JSON file.")
                (Args::AS_OUT_DIR.c_str(), bpo::value< std::string >(&args.outDir)->required(), "Output directory.")
                (Args::AS_PROJ_NUM_LIM.c_str(), bpo::value< int >(&args.projNumLimit)->default_value(3), "The limit number of points for a projection.")
                (Args::AS_PROJ_CURV_LIM.c_str(), bpo::value< float >(&args.projCurvLimit)->default_value(0.01), "The curvature limit for a point set to be considered as flat");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(
                Args::AS_IN_CLOUD.c_str(), 1
                ).add(Args::AS_IN_OCP_MAP.c_str(), 1
                ).add(Args::AS_IN_SETS_JSON.c_str(), 1
                ).add(Args::AS_IN_CAM_PROJS.c_str(), 1
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

struct BoundaryPointSets {
    BoundaryPointSets()
        : pEquivalentNormals( new pcl::PointCloud<pcl::PointNormal> )
    {}

    BoundaryPointSets( const BoundaryPointSets &other ) {
        *(this->pEquivalentNormals) = *(other.pEquivalentNormals);
        this->candidateSets = other.candidateSets;
    }

    BoundaryPointSets( BoundaryPointSets &&other ) noexcept {
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

    BoundaryPointSets& operator = ( BoundaryPointSets &&other ) noexcept {
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

            if ( static_cast<float>(count) / static_cast<float>(nIndices) >= threshold ) {
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
        } else {
            std::cout << "Boundary candidate " << i
                      << " is near an occlusion region. \n";
        }
    }

    std::cout << filteredBPS.candidateSets.size() << " / " << N
              << " boundary candidate point sets remain after filtering by the occupancy map. \n";

    QUICK_TIME_SHOW(te, "Filter boundary point sets by the occupancy map")

    return filteredBPS;
}

static std::vector< CameraProjection<float> >
        read_cam_projs( const std::string &fn ) {
    auto camProjs = read_cam_proj_from_json<float>(fn);

    // Test use.
    std::cout << camProjs.size() << " camera projection objects read from JSON file. \n";
    std::cout << "camProjs[last] = \n"
              << camProjs[ camProjs.size() - 1 ] << "\n";

    return camProjs;
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

    // Read the JSON file.
    BoundaryPointSets boundaryPointSets;
    pcu::read_equivalent_normal_from_json( args.inSets,
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
    print_bar("Read CameraProjection objects.");
    std::vector< CameraProjection<float> > camProj =
            read_cam_projs(args.inCamProjs);

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